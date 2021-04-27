#include <ros/ros.h>

#include <algorithm>

#include <simple_moveit_wrapper/industrial_robot.h>

#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>

#include <ocpl_sampling/grid_sampler.h>
#include <ocpl_sampling/halton_sampler.h>
#include <ocpl_sampling/random_sampler.h>
#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>
#include <ocpl_planning/types.h>

#include <ocpl_ros/planning_cases/l_profile.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

const std::string THIS_PACKAGE_NAME = "ocpl_benchmark_scripts";

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    simple_moveit_wrapper::IndustrialRobot robot("manipulator", "tool_tip");

    //////////////////////////////////
    // Load task
    //////////////////////////////////
    auto task = l_profile::waypoints(/* x_offset */ 0.98, /* extra_tolerance */ true);

    //////////////////////////////////
    // Translate robot kinematics to solver interface
    //////////////////////////////////
    // function that tells you whether a state is valid (collision free)
    auto f_is_valid = [&robot](const JointPositions& q) { return !robot.isColliding(q); };

    // function that returns analytical inverse kinematics solution for end-effector pose
    auto f_ik = [&robot](const Transform& tf, const JointPositions& /*q_fixed*/) { return robot.ik(tf); };

    // covert simple_moveit_wrapper limits to ocpl limits
    // (they are the exactly the same struct, but I'm not sure how to share the common data type)
    JointLimits joint_limits;
    for (auto bound : robot.getJointPositionLimits())
    {
        joint_limits.push_back(ocpl::Bounds{ bound.lower, bound.upper });
    }

    Robot bot{ robot.getNumDof(),
               robot.getNumRedDof(),
               joint_limits,
               [&robot](const JointPositions& q) { return robot.fk(q); },
               f_ik,
               f_is_valid };

    auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
        Eigen::VectorXd dv = poseDistance(tsr.tf_nominal, robot.fk(q));
        // only penalize the x and y rotation
        return std::sqrt(dv[3] * dv[3] + dv[4] * dv[4]);
    };
    //////////////////////////////////
    // Visualize a single solve
    //////////////////////////////////
    // Rviz rviz;
    // rviz.clear();

    // EigenSTL::vector_Vector3d points;
    // for (auto& pt : task)
    // {
    //     rviz.plotPose(pt.tf_nominal);
    //     points.push_back(pt.tf_nominal.translation());
    // }
    // rviz.visual_tools_->publishPath(points);
    // ros::Duration(0.1).sleep();

    // auto ps = loadSettingsFromFile("benchmark7/halton_gs.yaml", THIS_PACKAGE_NAME);

    // UnifiedPlanner planner(bot, ps);

    // ROS_INFO("Solving a single task.");
    // Solution solution = planner.solve(task, norm2Diff, zeroStateCost);

    // robot.animatePath(rviz.visual_tools_, solution.path);

       ////////////////////////////////
    // Benchmark specific parameter
    ////////////////////////////////
    // read the base settings from a file, to be modified later to execute parameter sweeps
    std::vector<std::string> file_names = readLinesFromFile("benchmark7/names.txt", THIS_PACKAGE_NAME);
    std::vector<PlannerSettings> base_settings;
    ROS_INFO("Running benchmark for the settings files:");
    for (auto name : file_names)
    {
        ROS_INFO_STREAM(name);
        base_settings.push_back(loadSettingsFromFile(name, THIS_PACKAGE_NAME));
    }

    // use the base settings to get a new settings vector that does the paramter sweeps
    // the minimum number of valid samples for the incremental methods
    std::vector<int> min_sample_range;
    for (auto s : readLinesFromFile("benchmark7/sample_settings.txt", THIS_PACKAGE_NAME))
    {
        if (s != "")
        {
            min_sample_range.push_back(std::stoi(s));
        }
    }

    std::vector<PlannerSettings> settings;
    for (auto setting : base_settings)
    {
        for (std::size_t i{ 0 }; i < min_sample_range.size(); ++i)
        {
            PlannerSettings new_setting = setting;

            new_setting.name = setting.name + "_" + std::to_string(min_sample_range[i]);
            new_setting.min_valid_samples = min_sample_range[i];
            new_setting.max_iters = 10 * new_setting.min_valid_samples;

            // fix everything back to random sampling
            new_setting.sampler_type = SamplerType::RANDOM;

            settings.emplace_back(new_setting);
        }
    }


    UnifiedPlanner planner(bot, base_settings.back());
    std::string outpath = ros::package::getPath(THIS_PACKAGE_NAME);
    outpath.append("/results/global_vs_local_");
    outpath.append("l_profile_random_obstacle");
    outpath.append(".csv");
    runBenchmark(outpath, bot, task, planner, settings, 30);

    return 0;
}
