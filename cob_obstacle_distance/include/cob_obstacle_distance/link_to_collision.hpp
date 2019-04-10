/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <stdint.h>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <urdf/model.h>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include <fcl/collision_object.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/collision_data.h>

#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>


#include "cob_obstacle_distance/marker_shapes/marker_shapes_interface.hpp"
#include "cob_obstacle_distance/marker_shapes/marker_shapes.hpp"
#include "cob_obstacle_distance/shapes_manager.hpp"
#include "cob_obstacle_distance/obstacle_distance_data_types.hpp"

class LinkToCollision
{
    private:
        typedef urdf::LinkConstSharedPtr PtrConstLink_t;
        typedef urdf::LinkSharedPtr PtrLink_t;
        typedef std::vector<PtrLink_t> VecPtrLink_t;
        typedef urdf::CollisionSharedPtr PtrCollision_t;
        typedef urdf::GeometrySharedPtr PtrGeometry_t;
        typedef urdf::MeshSharedPtr PtrMesh_t;
        typedef urdf::BoxSharedPtr PtrBox_t;
        typedef urdf::SphereSharedPtr PtrSphere_t;
        typedef urdf::CylinderSharedPtr PtrCylinder_t;
        typedef std::unordered_map<std::string, std::vector<std::string> >::iterator MapIter_t;

        urdf::Model model_;
        bool success_;
        std::string root_frame_id_;
        std::unordered_map<std::string, std::vector<std::string> > self_collision_map_; /// first: link to be considered 'obstacle', second: links of component to be considered for self-collision checking

        /**
         * Private method to create a specific marker shape for the output pointer.
         * @param link_of_interest The name of the link of interest.
         * @param pose The pose of the frame (root of the frame).
         * @param col The color.
         * @param geometry A pointer to a URDF geometry object.
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         */
        void createSpecificMarkerShape(const std::string& link_of_interest,
                                       const geometry_msgs::Pose& pose,
                                       const std_msgs::ColorRGBA& col,
                                       const PtrGeometry_t& geometry,
                                       PtrIMarkerShape_t& segment_of_interest_marker_shape);


        /**
         * Convert a pose object from URDF into a pose msg object.
         * @param urdf_pose Pose from URDF.
         * @param msg_pose Pose expressed as message.
         */
        void poseURDFToMsg(const urdf::Pose& urdf_pose, geometry_msgs::Pose& msg_pose);

    public:
        typedef std::unordered_map<std::string, std::vector<std::string> > MapSelfCollisions_t;

        LinkToCollision();
        ~LinkToCollision();

        inline MapSelfCollisions_t::iterator getSelfCollisionsIterBegin()
        {
            return this->self_collision_map_.begin();
        }

        inline MapSelfCollisions_t::iterator getSelfCollisionsIterEnd()
        {
            return this->self_collision_map_.end();
        }

        bool ignoreSelfCollisionPart(const std::string& link_of_interest, const std::string& self_collision_obstacle_link);

        /**
         * Initialize the FrameToCollision model by an URDF file.
         * @param root_frame_id The id of the root frame as base for further transformations.
         * @param urdf_file_name The URDF file name (containing path).
         * @return State of success.
         */
        bool initFile(const std::string& root_frame_id, const std::string& urdf_file_name);

        /**
         * Initialize the FrameToCollision model by an ROS parameter containting the URDF description.
         * @param root_frame_id The id of the root frame as base for further transformations.
         * @param urdf_param The ROS parameter containing the URDF description.
         * @return State of success.
         */
        bool initParameter(const std::string& root_frame_id, const std::string& urdf_param);

        /**
         * From the parameters self collision dictionary the keys are extracted as the self-collision "obstacles".
         * While the values are the parts to be ignored for self-collision checking.
         * Additionally the parts to be ignored are extended by the parent and child elements of the link corresponding to the key.
         * @param self_collision_params A XML RPC data structure representing the self_collision params (from YAML or parameter server)
         * @param sm The shapes manager that will by extended by self-collision "obstacles"
         * @return State of success.
         */
        bool initSelfCollision(XmlRpc::XmlRpcValue& self_collision_params, boost::scoped_ptr<ShapesManager>& sm);


        /**
         * Tries to find the given link_of_interest in the links parsed from URDF.
         * According to the data there a MarkerShape is created and will be assigned to the pointer.
         * @param abs_pos The absolute position (from root frame) of the shape.
         * @param quat_pos The orientation (from root frame) of the shape.
         * @param link_of_interest The name of the link of interest.
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         * @return State of success.
         */
        bool getMarkerShapeFromUrdf(const Eigen::Vector3d& abs_pos,
                                    const Eigen::Quaterniond& quat_pos,
                                    const std::string& link_of_interest,
                                    PtrIMarkerShape_t& segment_of_interest_marker_shape);

        /**
         * Tries to create a MarkerShape by a given shape_type.
         * If shape_type is MESH_RESOURCE then the behaviour is similar to getMarkerShapeFromUrdf(..).
         * @param shape_type The type of the shape (visualization_marker types).
         * @param abs_pos The absolute position (from root frame) of the shape.
         * @param quat_pos The orientation (from root frame) of the shape.
         * @param link_of_interest The name of the link of interest (e.g. link name if shape_type MESH_RESOURCE else only for id).
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         * @return State of success.
         */
        bool getMarkerShapeFromType(const uint32_t& shape_type,
                                    const Eigen::Vector3d& abs_pos,
                                    const Eigen::Quaterniond& quat_pos,
                                    const std::string& link_of_interest,
                                    const Eigen::Vector3d& dimension,
                                    PtrIMarkerShape_t& segment_of_interest_marker_shape);

        /**
         * Tries to create a MarkerShape by a given shape_type (similar to above but with pose).
         * @param shape_type The type of the shape (visualization_marker types).
         * @param pose The pose of the shape (with respect to the root_frame).
         * @param link_of_interest The name of the link of interest (e.g. link name if shape_type MESH_RESOURCE else only for id).
         * @param segment_of_interest_marker_shape The pointer for that a marker shape shall be created.
         * @return State of success.
         */
        bool getMarkerShapeFromType(const uint32_t& shape_type,
                                    const geometry_msgs::Pose& pose,
                                    const std::string& link_of_interest,
                                    const Eigen::Vector3d& dimension,
                                    PtrIMarkerShape_t& segment_of_interest_marker_shape);

};
