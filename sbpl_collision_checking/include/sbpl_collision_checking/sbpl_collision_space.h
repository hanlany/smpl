/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/** \author Benjamin Cohen */

#ifndef _SBPL_COLLISION_SPACE_
#define _SBPL_COLLISION_SPACE_

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <sbpl_collision_checking/bresenham.h>
#include <sbpl_collision_checking/occupancy_grid.h>
#include <sbpl_collision_checking/sbpl_collision_model.h>
#include <sbpl_geometry_utils/Interpolator.h>
#include <sbpl_geometry_utils/Voxelizer.h>
#include <sbpl_geometry_utils/SphereEncloser.h>
#include <leatherman/utils.h>
#include <tf_conversions/tf_kdl.h>
#include <angles/angles.h>

#include <arm_navigation_msgs/CollisionObject.h>
#include <geometry_msgs/Point.h>

using namespace std;

namespace sbpl_arm_planner{

/** @brief coords - used to pass around lists of valid cells */
typedef struct
{
  int x;
  int y;
  int z;
  bool bIsObstacle;
} CELL3V;

class SBPLCollisionSpace
{
  public:

    SBPLCollisionSpace(sbpl_arm_planner::OccupancyGrid* grid);

    ~SBPLCollisionSpace(){};

    bool init(std::string group_name);

    void setPadding(double padding);
    
    /** --------------- Collision Checking ----------- */
    void resetDistanceField();

    /** \brief check joint configuration for collision (0: collision) */
    bool checkCollision(const std::vector<double> &angles, bool verbose, bool visualize, unsigned char &dist);

    bool checkCollisionWithVisualizations(const std::vector<double> &angles, unsigned char &dist);

    /** \brief check linearly interpolated path for collisions */
    bool checkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, bool verbose, int &path_length, int &num_checks, unsigned char &dist);

    bool checkPathForCollisionWithVisualizations(const std::vector<double> &start, const std::vector<double> &end, bool verbose, unsigned char &dist);

    /** \brief check if a specific link is in collision (0: collision) */
    bool checkLinkForCollision(const std::vector<double> &angles, int link_num, bool verbose, unsigned char &dist);

    /** \brief check linearly interpolated path for collision of a specific link */
    bool checkLinkPathForCollision(const std::vector<double> &start, const std::vector<double> &end, int link_num, bool verbose, unsigned char &dist);

    /** \brief check if the cell's distance to nearest obstacle > radius */
    inline bool isValidCell(const int x, const int y, const int z, const int radius);

    /** \brief check if a line segment lies on an invalid cell (0: collision) */
    unsigned char isValidLineSegment(const std::vector<int> a,const std::vector<int> b,const int radius);

    bool getClearance(const std::vector<double> &angles, int num_spheres, double &avg_dist, double &min_dist);
    
    /** ---------------- Utils ---------------- */
    bool doesLinkExist(std::string name);

    /** \brief transform a pose from one frame to another */
    //void transformPose(std::string current_frame, std::string desired_frame, geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out);

    void transformPose(const std::string &current_frame, const std::string &desired_frame, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out);

    bool interpolatePath(const std::vector<double>& start, const std::vector<double>& end, std::vector<std::vector<double> >& path);
    
    bool interpolatePath(const std::vector<double>& start, const std::vector<double>& end, const std::vector<double>& inc, std::vector<std::vector<double> >& path);
    
    /** \brief get coordinates of cells that a line segment intersects */
    void getLineSegment(const std::vector<int> a,const std::vector<int> b,std::vector<std::vector<int> > &points);

    /** ------------ Kinematics ----------------- */
    std::string getGroupName() { return group_name_; };
   
    std::string getReferenceFrame() { return model_.getReferenceFrame(group_name_); };

    void setJointPosition(std::string name, double position);

    bool setPlanningJoints(const std::vector<std::string> &joint_names);

    bool getCollisionSpheres(const std::vector<double> &angles, std::vector<std::vector<double> > &spheres);

    /* ------------- Collision Objects -------------- */
    void addCubeToGrid(double x, double y, double z, double dim_x, double dim_y, double dim_z);
    void addCubesToGrid(std::vector<std::vector<double> > &cubes);
    void addCollisionObject(const arm_navigation_msgs::CollisionObject &object);
    void removeCollisionObject(const arm_navigation_msgs::CollisionObject &object);
    void processCollisionObjectMsg(const arm_navigation_msgs::CollisionObject &object);
    void removeAllCollisionObjects();
    void putCollisionObjectsInGrid();
    void getCollisionObjectVoxelPoses(std::vector<geometry_msgs::Pose> &points);
    void getOccupiedVoxels(double x_center, double y_center, double z_center, double radius, std::string text, std::vector<geometry_msgs::Point> &voxels);
    
    /** --------------- Attached Objects --------------*/
    void attachSphere(std::string name, std::string link, geometry_msgs::Pose pose, double radius);
    void attachCylinder(std::string link, geometry_msgs::Pose pose, double radius, double length);
    void attachCube(std::string name, std::string link, geometry_msgs::Pose pose, double x_dim, double y_dim, double z_dim);
    void attachMesh(std::string name, std::string link, geometry_msgs::Pose pose, const std::vector<geometry_msgs::Point> &vertices, const std::vector<int> &triangles);

    void removeAttachedObject();
    bool getAttachedObject(const std::vector<double> &angles, std::vector<std::vector<double> > &xyz);
   
    /** --------------- Debugging ----------------*/
    int num_collision_checks_; 
    int num_false_collision_checks_;
    std::vector<sbpl_arm_planner::Sphere> collision_spheres_;
    std::vector<sbpl_arm_planner::Sphere> attached_collision_spheres_;


  private:

    sbpl_arm_planner::SBPLCollisionModel model_;
    sbpl_arm_planner::OccupancyGrid* grid_;

    /* ----------- Parameters ------------ */
    double padding_;
    std::string group_name_;
    double object_enclosing_sphere_radius_;

    /* ----------- Robot ------------ */
    std::vector<std::string> planning_joints_;
    std::vector<double> inc_;
    std::vector<double> min_limits_;
    std::vector<double> max_limits_;
    std::vector<bool> continuous_;
    std::vector<Sphere*> spheres_; // temp
    std::vector<std::vector<KDL::Frame> > frames_; // temp

    /* ------------- Collision Objects -------------- */
    std::vector<std::string> known_objects_;
    std::map<std::string, arm_navigation_msgs::CollisionObject> object_map_;
    std::map<std::string, std::vector<Eigen::Vector3d> > object_voxel_map_;

    /** --------------- Attached Objects --------------*/
    bool object_attached_;
    int attached_object_frame_num_;
    int attached_object_segment_num_;    
    int attached_object_chain_num_;
    std::string attached_object_frame_;
    std::vector<Sphere> object_spheres_;
};

inline bool SBPLCollisionSpace::isValidCell(const int x, const int y, const int z, const int radius)
{
  if(grid_->getCell(x,y,z) <= radius)
    return false;
  return true;
}

} 
#endif
