////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2012, Benjamin Cohen
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen

// standard includes
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>
#include <numeric>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>
#include <smpl/ros/planner_interface.h>
#include <smpl/distance_map/edge_euclid_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <sbpl_kdl_robot_model/kdl_robot_model.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/angles.h>
#include <smpl/debug/visualizer_ros.h>

#include "collision_space_scene_multithread.h"
#include "pr2_allowed_collision_pairs.h"
#include "franka_allowed_collision_pairs.h"

#define VERBOSE 0

void FillGoalConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

//    goals.position_constraints[0].position.x = pose[0];
//    goals.position_constraints[0].position.y = pose[1];
//    goals.position_constraints[0].position.z = pose[2];

    Eigen::Quaterniond q;
    smpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

auto GetCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id)
    -> moveit_msgs::CollisionObject
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

auto GetCollisionCubes(
    std::vector<std::vector<double>>& objects,
    std::vector<std::string>& object_ids,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

auto GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

bool ReadInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state)
{
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                }
                else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    }
    else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    smpl::angles::from_euler_zyx(
                            (double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    ROS_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

struct RobotModelConfig
{
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config)
{
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    return true;
}

struct PlannerConfig
{
    std::string discretization;
    std::string mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
};

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

auto SetupRobotModel(const std::string& urdf, const RobotModelConfig &config, int num_threads)
    -> std::shared_ptr<smpl::KDLRobotModel>
{
    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
        return NULL;
    }

    ROS_INFO("Construct Generic KDL Robot Model");
    std::shared_ptr<smpl::KDLRobotModel> rm(new smpl::KDLRobotModel);

    if (!rm->init(urdf, config.kinematics_frame, config.chain_tip_link, num_threads)) {
        ROS_ERROR("Failed to initialize robot model.");
        return NULL;
    }

    return rm;
}


std::vector<std::pair<std::string, std::vector<double>>> get_goals(std::vector<moveit_msgs::CollisionObject>& collision_objects)
{
    std::vector<std::pair<std::string, std::vector<double>>> name_pose;
    std::unordered_map<std::string, std::vector<double>> goals;



    double height = 0.6;
    double y_offset = -0.25;
    goals["shelf_1"] = {0.4, -0.85 + y_offset, height + 0.15, 0, 0, 0};
    goals["shelf_2"] = {0.4, -0.55 + y_offset, height + 0.15, 0, 0, 0};
    goals["shelf_3"] = {0.4, -0.25 + y_offset, height + 0.15, 0, 0, 0};
    goals["shelf_4"] = {0.4, -0.05 + y_offset, height + 0.15, 0, 0, 0};
    goals["shelf_5"] = {0.4, 0.35 + y_offset, height + 0.15, 0, 0, 0};

    // Bottom row, -y -> +y
    goals["shelf_6"] = {0.4, -0.85 + y_offset, height-0.15, 0, 0, 0};
    goals["shelf_7"] = {0.4, -0.55 + y_offset, height-0.15, 0, 0, 0};
    goals["shelf_8"] = {0.4, -0.25 + y_offset, height-0.15, 0, 0, 0};
    goals["shelf_9"] = {0.4, -0.05 + y_offset, height-0.15, 0, 0, 0};
    goals["shelf_10"] = {0.4, 0.35 + y_offset, height-0.15, 0, 0, 0};
    goals["shelf_11"] = {0.4, 0.25 + y_offset, height-0.15, 0, 0, 0};
    goals["shelf_12"] = {0.4, -0.25 + y_offset, height + 0.15, 0, 0, 0};
    goals["shelf_13"] = {0.4, -0.25 + y_offset, height - 0.15, 0, 0, 0};
    goals["shelf_14"] = {0.4, 0.10 + y_offset, height - 0.15, 0, 0, 0};

    // name_pose.emplace_back(std::make_pair("shelf_1", goals["shelf_1"]));
    name_pose.emplace_back(std::make_pair("shelf_2", goals["shelf_2"]));
    name_pose.emplace_back(std::make_pair("shelf_8", goals["shelf_8"]));
    name_pose.emplace_back(std::make_pair("shelf_3", goals["shelf_3"]));
    name_pose.emplace_back(std::make_pair("shelf_5", goals["shelf_5"]));
    // name_pose.emplace_back(std::make_pair("shelf_6", goals["shelf_6"]));

    // Bottom row, -y -> +y
    // name_pose.emplace_back(std::make_pair("shelf_7", goals["shelf_7"]));
    name_pose.emplace_back(std::make_pair("shelf_9", goals["shelf_9"]));
    name_pose.emplace_back(std::make_pair("shelf_11", goals["shelf_11"]));
    name_pose.emplace_back(std::make_pair("shelf_12", goals["shelf_12"]));
    name_pose.emplace_back(std::make_pair("shelf_13", goals["shelf_13"]));
    name_pose.emplace_back(std::make_pair("shelf_4", goals["shelf_4"]));
    name_pose.emplace_back(std::make_pair("shelf_14", goals["shelf_14"]));

    // name_pose.emplace_back(std::make_pair("shelf_12", goals["shelf_12"]));


    return name_pose;

}


void run_experiments(int num_threads, std::string planner_name,
    ros::NodeHandle& nh, ros::NodeHandle& ph, 
    smpl::PlanningParams& planner_params, 
    smpl::OccupancyGrid& grid,
    RobotModelConfig& robot_config, moveit_msgs::RobotState& start_state, 
    CollisionSpaceSceneMultithread& cs_scene,
    smpl::collision::CollisionSpaceMultithread& cc,
    std::shared_ptr<smpl::KDLRobotModel> rm, 
    const std::string& planning_frame, std::vector<moveit_msgs::CollisionObject>& collision_objects)
{

    auto goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal", 10);


    auto goals = get_goals(collision_objects);

    double allowed_planning_time;
    ph.param("allowed_planning_time", allowed_planning_time, 10.0);

    std::vector<bool> success(goals.size(), false);
    std::vector<double> planning_times(goals.size(), -1);
    std::vector<double> solution_costs(goals.size(), -1);
    std::vector<int> state_expansions(goals.size(), -1);
    std::vector<int> edge_expansions(goals.size(), -1);
    std::vector<int> edge_expansions_rate(goals.size(), -1);

    std::vector<std::vector<double>> starts = 
    {{0, 0, 0, -1.1356 ,0, -1.05, 0},
    {-1.15494, 0.358322, -1.39402, -0.547191, -0.0360203, -0.635847, 1.20247},
    {-0.816889, 1.11682, -0.597626, -0.68616, -0.459279, -0.618735, 0.641153},
    {-1.21098, 0.730449, -1.32468, -1.5694, 3.14159, -0.263978, -2.33501},
    {0.561873, 1.17091, -0.0124739, -1.69102, 2.22937, -0.755835, -2.38112},
    {-0.828813, 1.18254, -1.2943, -0.97703, 0.960312, -0.510405, -0.486079},
    {0.0883643, 1.2963, -0.604049, -0.942974, 1.23355, -0.713154, -0.940397},
    {-1.21742, 0.703034, -1.36426, -1.56792, 3.04086, -0.26581, -2.20094},
    {-0.955761, 0.980127, -1.25805, -0.661308, 0.367367, -0.606987, 0.265894},
    {-0.868459, 1.08927, -1.07149, -1.84564, 2.89501, -0.595506, -2.51283}};



    for (int idx = 0; idx < goals.size(); ++idx)
    {


        auto goal = goals[idx].second;
        auto start = starts[idx];

        geometry_msgs::PoseStamped goal_pose;
        goal_pose.header.frame_id = planning_frame;
        goal_pose.pose.position.x = goal[0];
        goal_pose.pose.position.y = goal[1];
        goal_pose.pose.position.z = goal[2];
        
        int i = 0;
        // while(true)
        while(i < 4)
        {
            goal_pub.publish(goal_pose);
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            i++;
        }


        for (int i = 0; i < start_state.joint_state.name.size(); ++i)
        {
            if (start_state.joint_state.name[i] == "r_shoulder_pan_joint")
                start_state.joint_state.position[i] = start[0];

            if (start_state.joint_state.name[i] == "r_shoulder_lift_joint")
                start_state.joint_state.position[i] = start[1];

            if (start_state.joint_state.name[i] == "r_upper_arm_roll_joint")
                start_state.joint_state.position[i] = start[2];

            if (start_state.joint_state.name[i] == "r_elbow_flex_joint")
                start_state.joint_state.position[i] = start[3];

            if (start_state.joint_state.name[i] == "r_forearm_roll_joint")
                start_state.joint_state.position[i] = start[4];

            if (start_state.joint_state.name[i] == "r_wrist_flex_joint")
                start_state.joint_state.position[i] = start[5];

            if (start_state.joint_state.name[i] == "r_wrist_roll_joint")
                start_state.joint_state.position[i] = start[6];                
        }

        // Set reference state in the robot planning model...
        smpl::urdf::RobotState reference_state;
        InitRobotState(&reference_state, &rm->m_robot_model);
        for (auto i = 0; i < start_state.joint_state.name.size(); ++i) {
            auto* var = GetVariable(&rm->m_robot_model, &start_state.joint_state.name[i]);
            if (var == NULL) {
                ROS_WARN("Failed to do the thing");
                continue;
            }
            if (VERBOSE)
                ROS_INFO("Set joint %s to %f", start_state.joint_state.name[i].c_str(), start_state.joint_state.position[i]);
            SetVariablePosition(&reference_state, var, start_state.joint_state.position[i]);
        }
        SetReferenceState(rm.get(), GetVariablePositions(&reference_state));

        // Set reference state in the collision model...
        for (auto tidx = 0; tidx < num_threads; ++tidx)
        {
            if (!cs_scene.SetRobotState(tidx, start_state)) {
                ROS_ERROR("Failed to set start state on Collision Space Scene");
            }

            cc.setWorldToModelTransform(tidx, Eigen::Affine3d::Identity());
        }

        moveit_msgs::MotionPlanRequest req;
        moveit_msgs::MotionPlanResponse res;
        
        ph.param("allowed_planning_time", req.allowed_planning_time, 10.0);
        
        req.goal_constraints.resize(1);
        FillGoalConstraint(goal, planning_frame, req.goal_constraints[0]);
        req.group_name = robot_config.group_name;
        req.max_acceleration_scaling_factor = 1.0;
        req.max_velocity_scaling_factor = 1.0;
        req.num_planning_attempts = 1;
        req.planner_id = planner_name + ".bfs.manip";
        req.start_state = start_state;

        // Set up planner interface
        auto planner_interface = std::make_shared<smpl::PlannerInterface>(rm.get(), &cc, &grid);

        if (!planner_interface->init(planner_params)) 
        {
            ROS_ERROR("Failed to initialize Planner Interface");
        }

        // plan
        if (VERBOSE)
            ROS_INFO("Calling solve...");
       
        moveit_msgs::PlanningScene planning_scene;
        planning_scene.robot_state = start_state;


        auto plan_found = planner_interface->solve(planning_scene, req, res);
        auto planning_stats = planner_interface->getPlannerStats();

        planning_times[idx] = planning_stats["initial solution planning time"];
        solution_costs[idx] = planning_stats["solution cost"];
        state_expansions[idx] = planning_stats["state_expansions"];
        edge_expansions[idx] = planning_stats["edge_expansions"];
        edge_expansions_rate[idx] = edge_expansions[idx]/planning_times[idx];

        if ((!plan_found) || (res.trajectory.joint_trajectory.points.size() == 0))
        {
            // ROS_ERROR("Failed to plan.");
            success[idx] = false;
            solution_costs[idx] = -1;
        }    
        else
        {
            if (res.trajectory.joint_trajectory.points.size() == 0)
            {
                ROS_ERROR("Failed to plan.");
            }
            else
            {
                auto planning_stats = planner_interface->getPlannerStats();
                
                if (VERBOSE)
                {
                    ROS_INFO("Planning statistics");
                    if (VERBOSE)
                        for (auto& entry : planning_stats)
                                ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
                }

                success[idx] = true;                

                // size_t pidx = 0;
                // while (ros::ok()) {
                //     auto& point = res.trajectory.joint_trajectory.points[pidx];
                //     auto markers = cc.getCollisionRobotVisualization(0, point.positions);
                //     for (auto& m : markers.markers) {
                //         m.ns = "path_animation";
                //     }
                //     SV_SHOW_INFO(markers);
                //     std::this_thread::sleep_for(std::chrono::milliseconds(200));
                //     pidx++;
                //     pidx %= res.trajectory.joint_trajectory.points.size();
                // }
                // 
                // start = res.trajectory.joint_trajectory.points.back().positions;
            }

        }

        // std::cin.get();
    }


    for (int i = 0; i < goals.size(); ++i)
    {
        std::cout << "Exp " << i 
        << " | success: " << success[i] 
        << " | time: " << planning_times[i] 
        << " | edges: " << edge_expansions[i]
        << " | edge evaluations rate: " << edge_expansions_rate[i]
        << " | cost: " << solution_costs[i]
        << std::endl;
    }


    std::vector<double> success_costs, success_times;

    for (int i = 0; i < goals.size(); ++i)
    {
        if (solution_costs[i] != -1)
        {
            success_times.emplace_back(planning_times[i]);
            success_costs.emplace_back(solution_costs[i]);
        }
    }


    std::cout << "--------------------------------" << std::endl;
    std::cout << "Success rate: " << double(success_times.size())/planning_times.size() << std::endl;
    std::cout << "Mean success time: " << std::accumulate(success_times.begin(), success_times.end(), 0.0)/success_times.size() << std::endl;
    std::cout << "Mean time: " << std::accumulate(planning_times.begin(), planning_times.end(), 0.0)/planning_times.size() << std::endl;
    std::cout << "Mean number of edge evaluations: " << std::accumulate(edge_expansions.begin(), edge_expansions.end(), 0.0)/edge_expansions.size() << std::endl;
    std::cout << "Mean rate of edge evaluations: " << std::accumulate(edge_expansions_rate.begin(), edge_expansions_rate.end(), 0.0)/edge_expansions_rate.size() << std::endl;
    std::cout << "Mean success cost: " << std::accumulate(success_costs.begin(), success_costs.end(), 0.0)/success_costs.size() << std::endl;
    std::cout << "--------------------------------" << std::endl;




}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");
    
    int num_threads;    
    std::string planner_name;

    if (argc == 3)
    {
        planner_name = argv[1];
        num_threads = atoi(argv[2]);        
    }
    else
    {
        planner_name = "arastar";
        num_threads = 1;
    }

    
    std::cout << "num_threads: " << num_threads << std::endl;

    ROS_INFO("Initialize visualizer");
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    // Let publishers set up
    ros::Duration(1.0).sleep();

    /////////////////
    // Robot Model //
    /////////////////

    ROS_INFO("Load common parameters");

    // Robot description required to initialize collision checker and robot
    // model...
    auto robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    std::string robot_description;
    if (!nh.getParam(robot_description_param, robot_description)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    // Everyone needs to know the name of the planning frame for reasons...
    // ...frame_id for the occupancy grid (for visualization)
    // ...frame_id for collision objects (must be the same as the grid, other than that, useless)
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    ROS_INFO("Initialize Occupancy Grid");

    auto df_size_x = 3.0;
    auto df_size_y = 3.0;
    auto df_size_z = 3.0;
    auto df_res = 0.02;
    auto df_origin_x = -0.75;
    auto df_origin_y = -1.5;
    auto df_origin_z = 0.0;
    auto max_distance = 1.8;

    using DistanceMapType = smpl::EuclidDistanceMap;

    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    auto ref_counted = false;
    smpl::OccupancyGrid grid(df, ref_counted);
    std::vector<smpl::OccupancyGrid*> grid_vec;

    grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initialize collision checker");

    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpaceMultithread instance.
    CollisionSpaceSceneMultithread scene;

    smpl::collision::CollisionModelConfig cc_conf;
    if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    smpl::collision::CollisionSpaceMultithread cc;
    if (!cc.init(
            num_threads,
            &grid,
            grid_vec,
            robot_description,
            cc_conf,
            robot_config.group_name,
            robot_config.planning_joints))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    if (cc.robotCollisionModel()->name() == "pr2") {
        smpl::collision::AllowedCollisionMatrix acm;
        for (auto& pair : PR2AllowedCollisionPairs) {
            acm.setEntry(pair.first, pair.second, true);
        }
        cc.setAllowedCollisionMatrix(acm);
    }

    if (cc.robotCollisionModel()->name() == "panda") {
        smpl::collision::AllowedCollisionMatrix acm;
        for (auto& pair : FrankaAllowedCollisionPairs) {
            acm.setEntry(pair.first, pair.second, true);
        }
        cc.setAllowedCollisionMatrix(acm);
    }

    /////////////////
    // Setup Scene //
    /////////////////

    ROS_INFO("Initialize scene");

    scene.SetCollisionSpace(&cc);

    std::string object_filename;
    ph.param<std::string>("object_filename", object_filename, "");


    std::vector<moveit_msgs::CollisionObject> objects;
    // Read in collision objects from file and add to the scene...
    if (!object_filename.empty()) {
        objects = GetCollisionObjects(object_filename, planning_frame);
        for (auto& object : objects) 
        {
            for (auto tidx = 0; tidx < num_threads; ++tidx)
                scene.ProcessCollisionObjectMsg(tidx, object);
        }
    }

    auto rm = SetupRobotModel(robot_description, robot_config, num_threads);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    // Read in start state from file and update the scene...
    // Start state is also required by the planner...
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 1;
    }

    // Set reference state in the robot planning model...
    smpl::urdf::RobotState reference_state;
    InitRobotState(&reference_state, &rm->m_robot_model);
    for (auto i = 0; i < start_state.joint_state.name.size(); ++i) {
        auto* var = GetVariable(&rm->m_robot_model, &start_state.joint_state.name[i]);
        if (var == NULL) {
            ROS_WARN("Failed to do the thing");
            continue;
        }
        ROS_INFO("Set joint %s to %f", start_state.joint_state.name[i].c_str(), start_state.joint_state.position[i]);
        SetVariablePosition(&reference_state, var, start_state.joint_state.position[i]);
    }
    SetReferenceState(rm.get(), GetVariablePositions(&reference_state));

    // Set reference state in the collision model...
    for (auto tidx = 0; tidx < num_threads; ++tidx)
    {
        if (!scene.SetRobotState(tidx, start_state)) {
            ROS_ERROR("Failed to set start state on Collision Space Scene");
            return 1;
        }

        cc.setWorldToModelTransform(tidx, Eigen::Affine3d::Identity());
    }


    // SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));

    SV_SHOW_INFO(cc.getCollisionRobotVisualization(0));
    SV_SHOW_INFO(cc.getCollisionWorldVisualization(0));
    SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());

    ///////////////////
    // Planner Setup //
    ///////////////////

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
        ROS_ERROR("Failed to read planner config");
        return 1;
    }


    smpl::PlanningParams params;

    params.addParam("num_threads", num_threads);
    params.addParam("discretization", planning_config.discretization);
    params.addParam("mprim_filename", planning_config.mprim_filename);
    params.addParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim);
    params.addParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim);
    params.addParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim);
    params.addParam("use_short_dist_mprims", planning_config.use_short_dist_mprims);
    params.addParam("xyz_snap_dist_thresh", planning_config.xyz_snap_dist_thresh);
    params.addParam("rpy_snap_dist_thresh", planning_config.rpy_snap_dist_thresh);
    params.addParam("xyzrpy_snap_dist_thresh", planning_config.xyzrpy_snap_dist_thresh);
    params.addParam("short_dist_mprims_thresh", planning_config.short_dist_mprims_thresh);
    params.addParam("epsilon", 100.0);
    params.addParam("search_mode", false);
    params.addParam("allow_partial_solutions", false);
    params.addParam("target_epsilon", 1.0);
    params.addParam("delta_epsilon", 1.0);
    params.addParam("improve_solution", false);
    params.addParam("bound_expansions", true);
    params.addParam("repair_time", 1.0);
    params.addParam("bfs_inflation_radius", 0.02);
    params.addParam("bfs_cost_per_cell", 100);

    //////////////
    // Planning //
    //////////////



    run_experiments(num_threads, planner_name, nh, ph, params, grid, robot_config, start_state, scene, cc, rm, planning_frame, objects);



    return 0;
}
