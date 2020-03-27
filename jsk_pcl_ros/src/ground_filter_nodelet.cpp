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
 *     disclaimer in the documentation and/o2r other materials provided
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

#include "jsk_pcl_ros/ground_filter.h"

#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{


  void SimpleGroundFilter::onInit()
  {
    z_max_ = 0.01;

    GroundFilter::onInit();
  }

  void SimpleGroundFilter::updateCondition()
  {
    ComparisonPtr gt (new Comparison("z", pcl::ComparisonOps::GT, z_max_));

    cond_->addComparison(gt);
   
    filter_instance_.setCondition(cond_);
  }

  
  /*** GroundFilter ***/
  template <class FComparison>
  GroundFilter<FComparison>::GroundFilter() : cond_(new pcl::ConditionAnd<pcl::PointXYZ>()) {}

  template <class FComparison>
  void GroundFilter<FComparison>::filter(const sensor_msgs::PointCloud2ConstPtr &input,
                              const PCLIndicesMsg::ConstPtr& indices)
  {

    boost::mutex::scoped_lock lock (mutex_);
    pcl::PointCloud<pcl::PointXYZ> tmp_in, tmp_out;
    sensor_msgs::PointCloud2 out;
    fromROSMsg(*input, tmp_in);

    filter_instance_.setInputCloud (tmp_in.makeShared());
    if (indices) {
      pcl::IndicesPtr vindices;
      vindices.reset(new std::vector<int> (indices->indices));
      filter_instance_.setIndices(vindices);
    }
    // This setCOndition needed but reason wakattenai
    filter_instance_.setCondition(cond_);
    filter_instance_.filter(tmp_out);
    if (tmp_out.points.size() > 0) {
      toROSMsg(tmp_out, out);
      pub_.publish(out);
    } else {
      NODELET_INFO("All points were removed");
    }
  }

  template <class FComparison>
  void GroundFilter<FComparison>::filter(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    filter(input, PCLIndicesMsg::ConstPtr());
  }

  template <class FComparison>
  void GroundFilter<FComparison>::onInit()
  {
    ConnectionBasedNodelet::onInit();

    updateCondition();
    bool keep_organized;
    pnh_->param("keep_organized", keep_organized, false);
    pnh_->param("use_indices", use_indices_, false);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    filter_instance_ = pcl::ConditionalRemoval<pcl::PointXYZ>(true);
    filter_instance_.setKeepOrganized(keep_organized);

    onInitPostProcess();
  }

  template <class FComparison>
  void GroundFilter<FComparison>::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    if (use_indices_) {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
      sub_indices_.subscribe(*pnh_, "indices", 1);
      sync_->connectInput(sub_input_, sub_indices_);
      sync_->registerCallback(boost::bind(&GroundFilter::filter, this, _1, _2));
      //sub_input_ = pnh_->subscribe("input", 1, &SimpleGroundFilter::filter, this);
    }
    else {
      sub_input_.registerCallback(&GroundFilter::filter, this);
      NODELET_INFO("subscribe");
    }
  }

  template <class FComparison>
  void GroundFilter<FComparison>::unsubscribe()
  {
    sub_input_.unsubscribe();
    if (use_indices_) {
      sub_indices_.unsubscribe();
    }
  }
  
}

typedef jsk_pcl_ros::SimpleGroundFilter SimpleGroundFilter;
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::SimpleGroundFilter, nodelet::Nodelet);

