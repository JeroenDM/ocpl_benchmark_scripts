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

int main(int argc, char** argv)
{
    // ros specific setup
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Select the correct type of robot
    // All robots in this script are assumed to be planar
    simple_moveit_wrapper::PlanarRobot robot("manipulator", "tool_tip");
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
    // Benchmark specific parameter
    //////////////////////////////////
    // read the base settings from a file, to be modified later to execute parameter sweeps
    std::vector<std::string> file_names = readLinesFromFile("sp/names.txt");
    std::vector<PlannerSettings> base_settings;
    ROS_INFO("Running benchmark for the settings files:");
    for (auto name : file_names)
    {
        ROS_INFO_STREAM(name);
        base_settings.push_back(loadSettingsFromFile(name));
    }

    // use the base settings to get a new settings vector that does the paramter sweeps
    // the minimum number of valid samples for the incremental methods
    std::vector<int> min_sample_range;
    for (auto s : readLinesFromFile("sp/sample_settings.txt"))
    {
        if (s != "")
        {
            min_sample_range.push_back(std::stoi(s));
        }
    }
    // the grid size for fixed resolution methods
    // this is experimentally determined to a get similar number of valid samples / waypoint
    std::vector<std::vector<int>> grid_sizes{};
    for (auto s : readLinesFromFile("sp/grid_settings.txt"))
    {
        if (s != "")
        {
            grid_sizes.push_back(stringToVector<int>(s));
            std::cout << s << " len: " << grid_sizes.back().size() << std::endl;
            assert(grid_sizes.back().size() == 4);
        }
    }

    std::vector<PlannerSettings> settings;
    for (auto setting : base_settings)
    {
        if (setting.sampler_type == SamplerType::HALTON || setting.sampler_type == SamplerType::RANDOM)
        {
            for (std::size_t i{ 0 }; i < min_sample_range.size(); ++i)
            {
                PlannerSettings new_setting = setting;
                // fixed grid + incremental sampler
                if (setting.max_iters == 1)
                {
                    int total = 25 * min_sample_range[i];
                    int ns = (int)std::round(std::pow((float)total, 0.5));
                    new_setting.name = setting.name + "_" + std::to_string(total);
                    new_setting.t_space_batch_size = ns;
                    new_setting.c_space_batch_size = ns;
                }
                // incremental sampling
                else
                {
                    new_setting.name = setting.name + "_" + std::to_string(min_sample_range[i]);
                    new_setting.min_valid_samples = min_sample_range[i];
                    new_setting.max_iters = 10 * new_setting.min_valid_samples;
                }
                settings.emplace_back(new_setting);
            }
        }
        // fix grid + uniform grid sampler
        else if (setting.sampler_type == SamplerType::GRID)
        {
            for (std::size_t i{ 0 }; i < grid_sizes.size(); ++i)
            {
                PlannerSettings new_setting = setting;
                int total = std::accumulate(grid_sizes[i].begin(), grid_sizes[i].end(), 1, std::multiplies<int>());
                new_setting.name = setting.name + "_" + std::to_string(total);
                new_setting.redundant_joints_resolution = { grid_sizes[i].at(0), grid_sizes[i].at(1),
                                                            grid_sizes[i].at(2) };
                new_setting.tsr_resolution = { 1, 1, 1, 1, 1, grid_sizes[i].at(3) };
                settings.emplace_back(new_setting);
            }
        }
    }

    UnifiedPlanner planner(bot, base_settings.back());
    // std::string outfilename{ "results/benchmark_halton_case_" };
    std::string outfilename{ "results/fixed_vs_incremental_case_" };
    outfilename.append("2_");      // planning case 2
    outfilename.append("sr.csv");  // sr = success rate results
    runBenchmark(outfilename, bot, task, planner, settings, 5);

    return 0;
}
