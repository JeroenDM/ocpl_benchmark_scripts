#include <ros/ros.h>

#include <algorithm>
#include <limits>
#include <chrono>
#include <cmath>
#include <cassert>
#include <numeric>

#include <simple_moveit_wrapper/planar_robot.h>

#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>
#include <ocpl_ros/planning_cases/case2.h>

#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/factories.h>
#include <ocpl_planning/cost_functions.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

const std::string THIS_PACKAGE_NAME = "ocpl_benchmark_scripts";

int main(int argc, char** argv)
{
    // ros specific setup
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Select the correct type of robot
    // All robots in this script are assumed to be planar
    simple_moveit_wrapper::PlanarRobot robot("manipulator", "tool0");
    auto task = case2::waypoints(20);
    std::vector<Bounds> tsr_bounds = task.at(0).bounds.asVector();  // use first waypoint bounds for all waypoints

    //////////////////////////////////
    // Setup the kinematic description
    //////////////////////////////////
    auto is_valid_fun = [&robot](const JointPositions& q) { return !robot.isColliding(q); };
    auto ik_fun = [&robot](const Transform& tf, const JointPositions& q_fixed) { return robot.ik(tf, q_fixed); };

    JointLimits jl;
    for (auto limit : robot.getJointPositionLimits())
    {
        jl.push_back(Bounds{ limit.lower, limit.upper });
    }
    Robot bot{ robot.getNumDof(), 3, jl, [&robot](const JointPositions& q) { return robot.fk(q); }, ik_fun, is_valid_fun };

    //////////////////////////////////
    // Visualize a single solve
    //////////////////////////////////
    Rviz rviz;
    rviz.clear();

    EigenSTL::vector_Vector3d points;
    for (auto& pt : task)
    {
        rviz.plotPose(pt.tf_nominal);
        points.push_back(pt.tf_nominal.translation());
    }
    rviz.visual_tools_->publishPath(points);
    ros::Duration(0.1).sleep();

    auto ps = loadSettingsFromFile("benchmark6/halton_ls.yaml", THIS_PACKAGE_NAME);

    UnifiedPlanner planner(bot, ps);
    std::reverse(task.begin(), task.end());

    ROS_INFO("Solving a single task.");
    Solution solution = planner.solve(task, norm2Diff, zeroStateCost);

    robot.animatePath(rviz.visual_tools_, solution.path);
    // savePath("last_path.npy", solution.path);

    ////////////////////////////////
    // Benchmark specific parameter
    ////////////////////////////////
    // read the base settings from a file, to be modified later to execute parameter sweeps
    // std::vector<std::string> file_names = readLinesFromFile("benchmark6/names.txt", THIS_PACKAGE_NAME);
    // std::vector<PlannerSettings> base_settings;
    // ROS_INFO("Running benchmark for the settings files:");
    // for (auto name : file_names)
    // {
    //     ROS_INFO_STREAM(name);
    //     base_settings.push_back(loadSettingsFromFile(name, THIS_PACKAGE_NAME));
    // }

    // // use the base settings to get a new settings vector that does the paramter sweeps
    // // the minimum number of valid samples for the incremental methods
    // std::vector<int> min_sample_range;
    // for (auto s : readLinesFromFile("benchmark6/sample_settings.txt", THIS_PACKAGE_NAME))
    // {
    //     if (s != "")
    //     {
    //         min_sample_range.push_back(std::stoi(s));
    //     }
    // }

    // std::vector<PlannerSettings> settings;
    // for (auto setting : base_settings)
    // {
    //     for (std::size_t i{ 0 }; i < min_sample_range.size(); ++i)
    //     {
    //         PlannerSettings new_setting = setting;

    //         new_setting.name = setting.name + "_" + std::to_string(min_sample_range[i]);
    //         new_setting.min_valid_samples = min_sample_range[i];
    //         new_setting.max_iters = 10 * new_setting.min_valid_samples;

    //         // fix everything back to random sampling
    //         new_setting.sampler_type = SamplerType::RANDOM;

    //         settings.emplace_back(new_setting);
    //     }
    // }

    // std::reverse(task.begin(), task.end());

    // UnifiedPlanner planner(bot, base_settings.back());
    // std::string outpath = ros::package::getPath(THIS_PACKAGE_NAME);
    // outpath.append("/results/global_vs_local_");
    // outpath.append("small_passage_random_out");
    // outpath.append(".csv");
    // runBenchmark(outpath, bot, task, planner, settings, 30);

    return 0;
}
