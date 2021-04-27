#include <ros/ros.h>

#include <algorithm>
#include <numeric>

#include <simple_moveit_wrapper/industrial_robot.h>
#include <ocpl_ros/rviz.h>
#include <ocpl_ros/io.h>
#include <ocpl_ros/planning_cases/text.h>

#include <ocpl_tsr/task_space_regions.h>

#include <ocpl_planning/acro_planner.h>
#include <ocpl_planning/io.h>
#include <ocpl_planning/math.h>

#include <ocpl_benchmark/benchmark.h>

using namespace ocpl;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ocpl_moveit_demo_rn");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    simple_moveit_wrapper::IndustrialRobot robot("manipulator", "tool_tip");
    simple_moveit_wrapper::Robot table("rotation_table", "work");
    auto work_pose = table.fk({ 0.0 });

    // Rviz rviz;
    // rviz.clear();

    //////////////////////////////////
    // Create task
    //////////////////////////////////
    ///////////////////////////////////////
    // hello text imported with header file
    ///////////////////////////////////////
    std::vector<TSR> task = text::waypoints(work_pose);

    // for (auto& pt : task)
    // {
    //     rviz.plotPose(pt.tf_nominal);
    // }
    // ros::Duration(0.1).sleep();

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

    // arc welding specific state cost
    // penalize deviation for x and y rotation
    auto state_cost_fun = [&robot](const TSR& tsr, const JointPositions& q) {
        auto v = tsr.poseToValues(robot.fk(q));
        return std::sqrt(v[3] * v[3] + v[4] * v[4]);
    };

    // auto ps = loadSettingsFromFile("benchmark3/grid.yaml");

    // UnifiedPlanner planner(bot, ps);

    // // // solve it!
    // ROS_INFO("Solving a single task.");
    // Solution solution = planner.solve(task, norm2Diff, state_cost_fun);

    // robot.animatePath(rviz.visual_tools_, solution.path);
    // savePath("last_path.npy", solution.path);

    //////////////////////////////////
    // Benchmark specific parameter
    //////////////////////////////////
    // read the base settings from a file, to be modified later to execute parameter sweeps
    std::vector<std::string> file_names = readLinesFromFile("benchmark3/names.txt");
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
    for (auto s : readLinesFromFile("benchmark3/sample_settings.txt"))
    {
        if (s != "")
        {
            min_sample_range.push_back(std::stoi(s));
        }
    }
    // the grid size for fixed resolution methods
    // this is experimentally determined to a get similar number of valid samples / waypoint
    std::vector<std::vector<int>> grid_sizes{};
    for (auto s : readLinesFromFile("benchmark3/grid_settings.txt"))
    {
        if (s != "")
        {
            grid_sizes.push_back(stringToVector<int>(s));
            std::cout << s << " len: " << grid_sizes.back().size() << std::endl;
            assert(grid_sizes.back().size() == 2);
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
                new_setting.redundant_joints_resolution = { grid_sizes[i].at(0) };
                new_setting.tsr_resolution = { 1, 1, 1, 1, 1, grid_sizes[i].at(1) };
                settings.emplace_back(new_setting);
            }
        }
    }

    UnifiedPlanner planner(bot, base_settings.back());
    // std::string outfilename{ "results/benchmark_halton_case_" };
    std::string outfilename{ "results/fixed_vs_incremental_case_" };
    outfilename.append("text_");   // planning case 2
    outfilename.append("sr_global.csv");  // sr = success rate results
    runBenchmark(outfilename, bot, task, planner, settings, 5);

    return 0;
}
