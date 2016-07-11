// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef JSK_PCL_ROS_PEOPLE_TRACKING_H_
#define JSK_PCL_ROS_PEOPLE_TRACKING_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/PeopleTrackingConfig.h"

#include <pcl/people/ground_based_people_detection_app.h>

namespace jsk_pcl_ros
{
  class PeopleTracking: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    PeopleTracking(): DiagnosticNodelet("PeopleTracking") { }

    typedef jsk_pcl_ros::PeopleTrackingConfig Config;

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();

    virtual void ground_coeffs_callback(
                                        const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg);
    virtual void set_ground_coeffs(const pcl_msgs::ModelCoefficients& coefficients);
    virtual void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void detect(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_cloud_;
    ros::Subscriber sub_info_;
    ros::Subscriber sub_coefficients_;

    ros::Publisher pub_camera_info_;
    ros::Publisher pub_box_;

    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    void configCallback (Config &config, uint32_t level);

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    boost::mutex mutex_;

    sensor_msgs::CameraInfo::ConstPtr latest_camera_info_;

    pcl::people::PersonClassifier<pcl::RGB> person_classifier_;
    pcl::people::GroundBasedPeopleDetectionApp<pcl::PointXYZRGBA> people_detector_;

    Eigen::VectorXf ground_coeffs_;

    double box_depth_;
    double box_width_;
    double min_confidence_;
    double people_height_threshold_;
    double voxel_size_;
    int queue_size_;
    std::string trained_filename_;
  private:

  };
}

#endif