#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>

#include <simple_moveit_wrapper/industrial_robot.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>
#include <ocpl_ros/planning_cases/halfopen_box.h>
#include <ocpl_ros/planning_cases/text.h>

#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/oriolo.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/io.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    simple_moveit_wrapper::IndustrialRobot robot("manipulator", "tool_tip");
    Rviz rviz;
    rviz.clear();

    simple_moveit_wrapper::Robot table("rotation_table", "work");
    auto work_pose = table.fk({ 0.0 });
    std::cout << "Work position: " << work_pose.translation().transpose() << "\n";

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    std::vector<TSR> task;
    std::vector<std::vector<TSR>> multiple_tasks;

    int PLANNING_CASE{ 1 };
    if (argc > 1)
    {
        PLANNING_CASE = std::stoi(argv[1]);
    }
    ROS_INFO_STREAM("Selected planning case " << PLANNING_CASE);

    switch (PLANNING_CASE)
    {
        case 1: {
            //////////////////////////////////
            // halfopen box from header file
            //////////////////////////////////
            task = halfopen_box::waypoints();
            for (auto& pt : task)
            {
                rviz.plotPose(pt.tf_nominal);
            }
            ros::Duration(0.1).sleep();
            break;
        }
        case 2: {
            ///////////////////////////////////////
            // hello text imported with header file
            ///////////////////////////////////////
            task = text::waypoints(work_pose);
            EigenSTL::vector_Vector3d path_positions;
            for (auto& pt : task)
            {
                path_positions.push_back(pt.tf_nominal.translation());
            }
            rviz.visual_tools_->publishPath(path_positions);
            rviz.visual_tools_->trigger();
            ros::Duration(0.1).sleep();
            break;
        }
        case 3: {
            //////////////////////////////////
            // from generic csv file
            //////////////////////////////////

            multiple_tasks = ocpl::readPathsFromCsvFile("halfopen_box.csv");
            task = multiple_tasks.at(2);

            ROS_INFO_STREAM("task length: " << task.size());

            EigenSTL::vector_Vector3d path_positions;
            for (auto& t : multiple_tasks)
            {
                path_positions.clear();
                for (auto& pt : t)
                {
                    path_positions.push_back(pt.tf_nominal.translation());
                }
                rviz.visual_tools_->publishPath(path_positions);
                rviz.visual_tools_->trigger();
                ros::Duration(0.1).sleep();
            }

            break;
        }
        default: {
            ROS_WARN("Unkown planning case id.");
            return 0;
        }
    }

    //////////////////////////////////
    // Simple interface solver
    //////////////////////////////////
    // function that tells you whether a state is valid (collision free)
    auto is_valid_fun = [&robot](const JointPositions& q) { return !robot.isColliding(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto ik_fun = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

    std::vector<ocpl::Bounds> joint_limits;
    auto jl_smw = robot.getJointPositionLimits();
    for (auto l : jl_smw)
    {
        joint_limits.push_back(ocpl::Bounds{ l.lower, l.upper });
    }

    Robot bot{ robot.getNumDof(),
               robot.getNumDof() - 6,
               joint_limits,
               [&robot](const JointPositions& q) { return robot.fk(q); },
               ik_fun,
               is_valid_fun };

    PlannerSettings ps = loadSettingsFromFile("halfopen_box1.yaml");

    auto path_cost_fun = L2NormDiff2;

    // optionally we can set a state cost for every point along the path
    // this one tries to keep the end-effector pose close the the nominal pose
    // defined in the task space region
    auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
        return poseDistance(tsr.tf_nominal, robot.fk(q)).norm();
    };
    // auto state_cost_fun = zeroStateCost;

    //////////////////////////////////
    // solve it!
    //////////////////////////////////
    // auto orioli_ps = loadOrioloSettings("oriolo1.txt");
    // oriolo::OrioloPlanner planner(bot, ps);

    ps = loadSettingsFromFile("kuka_on_rail/halton_incremental.yaml", "ocpl_benchmark_scripts");

    UnifiedPlanner planner(bot, ps);

    ROS_INFO("Solving a single task.");
    Solution solution = planner.solve(task, path_cost_fun, state_cost_fun);

    if (solution.success)
    {
        robot.animatePath(rviz.visual_tools_, solution.path);
        savePath("last_path.npy", solution.path);
    }

    return 0;
}
