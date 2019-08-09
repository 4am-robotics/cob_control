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


#include <string>
#include <vector>

#include <cob_obstacle_distance/link_to_collision.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <visualization_msgs/Marker.h>


LinkToCollision::LinkToCollision() : success_(true)
{
}


LinkToCollision::~LinkToCollision()
{
}


void LinkToCollision::poseURDFToMsg(const urdf::Pose& urdf_pose, geometry_msgs::Pose& msg_pose)
{
    msg_pose.position.x = urdf_pose.position.x;
    msg_pose.position.y = urdf_pose.position.y;
    msg_pose.position.z = urdf_pose.position.z;

    msg_pose.orientation.x = urdf_pose.rotation.x;
    msg_pose.orientation.y = urdf_pose.rotation.y;
    msg_pose.orientation.z = urdf_pose.rotation.z;
    msg_pose.orientation.w = urdf_pose.rotation.w;
}


bool LinkToCollision::ignoreSelfCollisionPart(const std::string& link_of_interest,
                                              const std::string& self_collision_obstacle_link)
{
    if (this->self_collision_map_.count(self_collision_obstacle_link) <= 0)
    {
        return false;
    }

    std::vector<std::string>::iterator sca_begin = this->self_collision_map_[self_collision_obstacle_link].begin();
    std::vector<std::string>::iterator sca_end = this->self_collision_map_[self_collision_obstacle_link].end();
    return std::find(sca_begin, sca_end, link_of_interest) == sca_end;  // if not found return true
}


bool LinkToCollision::initFile(const std::string& root_frame_id, const std::string& urdf_file_name)
{
    this->root_frame_id_ = root_frame_id;
    this->success_ = this->model_.initFile(urdf_file_name);
    return this->success_;
}


bool LinkToCollision::initParameter(const std::string& root_frame_id, const std::string& urdf_param)
{
    this->root_frame_id_ = root_frame_id;
    this->success_ = this->model_.initParam(urdf_param);
    return this->success_;
}


bool LinkToCollision::initSelfCollision(XmlRpc::XmlRpcValue& self_collision_params, boost::scoped_ptr<ShapesManager>& sm)
{
    bool success = true;
    ROS_ASSERT(self_collision_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    try
    {
        for (XmlRpc::XmlRpcValue::iterator it = self_collision_params.begin(); it != self_collision_params.end(); ++it)
        {
            std::vector<std::string> empty_vec;
            this->self_collision_map_[it->first] = empty_vec;
            ROS_ASSERT(it->second.getType() == XmlRpc::XmlRpcValue::TypeArray);
            for (int j=0; j < it->second.size(); ++j)
            {
                ROS_ASSERT(it->second[j].getType() == XmlRpc::XmlRpcValue::TypeString);
                this->self_collision_map_[it->first].push_back(it->second[j]);
            }
        }
    }
    catch(...)
    {
        success = false;
    }

    if (success)
    {
        for (MapIter_t it = this->self_collision_map_.begin(); it != this->self_collision_map_.end(); it++)
        {
            ROS_INFO_STREAM("Create self-collision obstacle for: " << it->first);

            // Create real obstacles now.
            PtrIMarkerShape_t ptr_obstacle;
            this->getMarkerShapeFromUrdf(Eigen::Vector3d(), Eigen::Quaterniond(), it->first, ptr_obstacle);
            ptr_obstacle->setDrawable(false);  // do not draw the marker in rviz again (are available in robot model->collision enabled...
            sm->addShape(it->first, ptr_obstacle);
        }
    }

    return success;
}


bool LinkToCollision::getMarkerShapeFromUrdf(const Eigen::Vector3d& abs_pos,
                                             const Eigen::Quaterniond& quat_pos,
                                             const std::string& link_of_interest,
                                             PtrIMarkerShape_t& segment_of_interest_marker_shape)
{
    if (!this->success_)
    {
        ROS_ERROR("FrameToCollision object has not been initialized correctly.");
        return false;
    }

    bool local_success = true;
    PtrConstLink_t link = this->model_.getLink(link_of_interest);
    if (NULL != link)
    {
        geometry_msgs::Pose pose;
        tf::pointEigenToMsg(abs_pos, pose.position);
        tf::quaternionEigenToMsg(quat_pos, pose.orientation);
        if (NULL != link->collision && NULL != link->collision->geometry)
        {
            this->poseURDFToMsg(link->collision->origin, pose);
            this->createSpecificMarkerShape(link_of_interest,
                                            pose,
                                            g_shapeMsgTypeToVisMarkerType.obstacle_color_,
                                            link->collision->geometry,
                                            segment_of_interest_marker_shape);
        }
        else if (NULL != link->visual && NULL != link->visual->geometry)
        {
            ROS_WARN_STREAM("Could not find a collision or collision geometry for " << link_of_interest <<
                            ". Trying to create the shape from visual.");
            this->poseURDFToMsg(link->visual->origin, pose);
            this->createSpecificMarkerShape(link_of_interest,
                                            pose,
                                            g_shapeMsgTypeToVisMarkerType.obstacle_color_,
                                            link->visual->geometry,
                                            segment_of_interest_marker_shape);
        }
        else
        {
            ROS_ERROR_STREAM("There is either no collision object or no collision geometry available: " << link_of_interest <<
                             ". Trying fallback solution: getMarker from a default SPHERE.");
            const Eigen::Vector3d dim(0.05, 0.1, 0.1);
            this->getMarkerShapeFromType(visualization_msgs::Marker::SPHERE,
                                         pose,
                                         link_of_interest,
                                         dim,
                                         segment_of_interest_marker_shape);
            local_success = segment_of_interest_marker_shape != NULL;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Could not find link in URDF model description: " << link_of_interest);
        local_success = false;
    }

    return local_success;
}


void LinkToCollision::createSpecificMarkerShape(const std::string& link_of_interest,
                                                const geometry_msgs::Pose& pose,
                                                const std_msgs::ColorRGBA& col,
                                                const PtrGeometry_t& geometry,
                                                PtrIMarkerShape_t& segment_of_interest_marker_shape)
{
    if (urdf::Geometry::MESH == geometry->type)
    {
        std_msgs::ColorRGBA test_col;
        test_col.a = 0.0;
        test_col.r = 0.0;

        PtrMesh_t mesh = std::static_pointer_cast<urdf::Mesh>(geometry);
        segment_of_interest_marker_shape.reset(new MarkerShape<BVH_RSS_t>(this->root_frame_id_,
                                                                          mesh->filename,
                                                                          pose,
                                                                          col));
    }
    else if (urdf::Geometry::BOX == geometry->type)
    {
        PtrBox_t urdf_box = std::static_pointer_cast<urdf::Box>(geometry);
        fcl::Box b(urdf_box->dim.x,
                   urdf_box->dim.y,
                   urdf_box->dim.z);

        std_msgs::ColorRGBA test_col;
        test_col.a = 1.0;
        test_col.r = 1.0;

        // segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Box>(this->root_frame_id_, b, pose, col));
        segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Box>(this->root_frame_id_, b, pose, test_col));
    }
    else if (urdf::Geometry::SPHERE == geometry->type)
    {
        std_msgs::ColorRGBA test_col;
        test_col.a = 1.0;
        test_col.b = 1.0;

        PtrSphere_t urdf_sphere = std::static_pointer_cast<urdf::Sphere>(geometry);
        fcl::Sphere s(urdf_sphere->radius);
        segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Sphere>(this->root_frame_id_, s, pose, test_col));
    }
    else if (urdf::Geometry::CYLINDER == geometry->type)
    {
        std_msgs::ColorRGBA test_col;
        test_col.a = 1.0;
        test_col.g = 1.0;

        PtrCylinder_t urdf_cyl = std::static_pointer_cast<urdf::Cylinder>(geometry);
        fcl::Cylinder c(urdf_cyl->radius, urdf_cyl->length);
        segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Cylinder>(this->root_frame_id_, c, pose, test_col));
    }
    else
    {
        ROS_ERROR_STREAM("Geometry type unknown: " << geometry->type);
    }
}


bool LinkToCollision::getMarkerShapeFromType(const uint32_t& shape_type,
                                             const Eigen::Vector3d& abs_pos,
                                             const Eigen::Quaterniond& quat_pos,
                                             const std::string& link_of_interest,
                                             const Eigen::Vector3d& dimension,
                                             PtrIMarkerShape_t& segment_of_interest_marker_shape)
{
    geometry_msgs::Pose pose;
    tf::pointEigenToMsg(abs_pos, pose.position);
    tf::quaternionEigenToMsg(quat_pos, pose.orientation);
    return this->getMarkerShapeFromType(shape_type,
                                        pose,
                                        link_of_interest,
                                        dimension,
                                        segment_of_interest_marker_shape);
}


bool LinkToCollision::getMarkerShapeFromType(const uint32_t& shape_type,
                                             const geometry_msgs::Pose& pose,
                                             const std::string& link_of_interest,
                                             const Eigen::Vector3d& dimension,
                                             PtrIMarkerShape_t& segment_of_interest_marker_shape)
{
    // Representation of segment_of_interest as specific fcl::Shape
    fcl::Box b(dimension(FCL_BOX_X), dimension(FCL_BOX_Y), dimension(FCL_BOX_Z));
    fcl::Sphere s(dimension(FCL_RADIUS));
    fcl::Cylinder c(dimension(FCL_RADIUS), dimension(FCL_CYL_LENGTH));
    uint32_t loc_shape_type = shape_type;
    std::string mesh_resource;
    if (visualization_msgs::Marker::MESH_RESOURCE == loc_shape_type)
    {
        PtrConstLink_t link = this->model_.getLink(link_of_interest);
        if (this->success_ &&
                NULL != link &&
                NULL != link->collision &&
                NULL != link->collision->geometry &&
                link->collision->geometry->type == urdf::Geometry::MESH)
        {
            PtrMesh_t mesh = std::static_pointer_cast<urdf::Mesh>(link->collision->geometry);
            mesh_resource = mesh->filename;
        }
        else
        {
            ROS_WARN_STREAM("Either link is not available in URDF or there is no MESH collision representation for " <<
                            link_of_interest << ". Rather using SPHERE.");
            loc_shape_type = visualization_msgs::Marker::SPHERE;
        }
    }

    std_msgs::ColorRGBA test_col;

    switch (loc_shape_type)
    {
        case visualization_msgs::Marker::CUBE:
            test_col.a = 1.0;
            test_col.r = 1.0;
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Box>(this->root_frame_id_, b, pose, test_col));
            break;
        case visualization_msgs::Marker::SPHERE:
            test_col.a = 1.0;
            test_col.b = 1.0;
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Sphere>(this->root_frame_id_, s, pose, test_col));
            break;
        case visualization_msgs::Marker::CYLINDER:
            test_col.a = 1.0;
            test_col.g = 1.0;
            segment_of_interest_marker_shape.reset(new MarkerShape<fcl::Cylinder>(this->root_frame_id_, c, pose, test_col));
            break;
        case visualization_msgs::Marker::MESH_RESOURCE:
            segment_of_interest_marker_shape.reset(new MarkerShape<BVH_RSS_t>(this->root_frame_id_,
                                                                              mesh_resource,
                                                                              pose,
                                                                              test_col));
            break;
        default:
           ROS_ERROR("Failed to process request due to unknown shape type: %d", loc_shape_type);
           return false;
    }

    return true;
}

