#include <fstream>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/timer.hpp>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

struct SelfCollisionData {
    std::vector<double> pt;
    bool collision;
};

static std::string const group_name = "right";
static std::string const ee_link_name = "/right/wam7";

int main(int argc, char **argv)
{
    if (argc != 2) {
        std::cerr << "error: Incorrect number of arguments.\n"
                     "usage: " << argv[0] << " /path/to/poses.yaml\n"
                  << std::endl;
        return 1;
    }
    std::string const input_path = argv[1];

    ros::init(argc, argv, "moveit_benchmark", ros::init_options::AnonymousName);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    planning_scene::PlanningScene planning_scene(kinematic_model);
    robot_state::RobotState &current_state = planning_scene.getCurrentStateNonConst();

    // Load poses from disk.
    std::vector<std::string> dof_names;
    std::vector<SelfCollisionData> poses;
    {
        std::ifstream in_stream(input_path.c_str(), std::ios::binary);
        YAML::Parser parser(in_stream);
        YAML::Node doc;
        parser.GetNextDocument(doc);

        doc["names"] >> dof_names;

        for (int i = 0; i < doc["data"].size(); ++i) {
            SelfCollisionData entry;
            doc["data"][i]["pose"] >> entry.pt;
            doc["data"][i]["collision"] >> entry.collision;
            poses.push_back(entry);
        }
    }
    std::cout << "Loaded " << poses.size() << " poses from '" << input_path << "'." << std::endl;

    // Pre-compute the mapping from joint names to DOFs.
    std::vector<robot_model::JointModel *> joints(dof_names.size());
    for (size_t i = 0; i < dof_names.size(); ++i) {
        joints[i] = kinematic_model->getJointModel(dof_names[i]);
        if (!joints[i]) {
            ROS_ERROR_STREAM("Unable to find joint '" << dof_names[i] << "'.");
            return 1;
        }
    }

    robot_model::JointModelGroup *manipulator_group = kinematic_model->getJointModelGroup(group_name);
    if (!manipulator_group) {
        ROS_ERROR_STREAM("Unable to find group '" << group_name << "'.\n");
        return 1;
    }

    // Run the benchmark.
    int num_checks = 50000;
    int num_collisions = 0;
    double setdofvalues_duration = 0.0;
    double fk_duration = 0.0;
    double jacobian_duration = 0.0;
    double collision_duration = 0.0;
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    BOOST_FOREACH (SelfCollisionData const &query, poses) {
        BOOST_ASSERT(query.pt.size() == dof_names.size());

        // Forward kinematics.
        boost::timer setdofvalues_timer;
        for (size_t i = 0; i < joints.size(); ++i) {
            current_state.setJointPositions(joints[i], &query.pt[i]);
        }
        setdofvalues_duration += setdofvalues_timer.elapsed();

        // Get the end-effector transform.
        boost::timer fk_timer;
        current_state.getFrameTransform(ee_link_name);
        fk_duration += fk_timer.elapsed();

        // Get the Jacobian.
        boost::timer jacobian_timer;
        current_state.getJacobian(manipulator_group);
        jacobian_duration += jacobian_timer.elapsed();

        // Self-collision checking.
        boost::timer collision_timer;
        collision_result.clear();
        planning_scene.checkSelfCollision(collision_request, collision_result);
        collision_duration += collision_timer.elapsed();

        if (collision_result.collision) {
            num_collisions++;
        }
    }

    std::cout << "Total number of queries: " << num_checks << '\n'
              << "setJointPositions:" << '\n'
              << "  Total ellapsed time: " << setdofvalues_duration << '\n'
              << "  Average time per check: " << (setdofvalues_duration / num_checks) << '\n'
              << "  Average checks per second: " << (num_checks / setdofvalues_duration) << '\n'
              << "getFrameTransform:" << '\n'
              << "  Total ellapsed time: " << fk_duration << '\n'
              << "  Average time per check: " << (fk_duration / num_checks) << '\n'
              << "  Average checks per second: " << (num_checks / fk_duration) << '\n'
              << "getJacobian" << '\n'
              << "  Total ellapsed time: " << jacobian_duration << '\n'
              << "  Average time per check: " << (jacobian_duration / num_checks) << '\n'
              << "  Average checks per second: " << (num_checks / jacobian_duration) << '\n'
              << "checkSelfCollision:" << '\n'
              << "  Total collisions found: " << num_collisions << '\n'
              << "  Total ellapsed time: " << collision_duration << '\n'
              << "  Average time per check: " << (collision_duration / num_checks) << '\n'
              << "  Average checks per second: " << (num_checks / collision_duration) << '\n'
              << std::flush;
    return 0;
}
