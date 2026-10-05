/*
 * Copyright (c) 2014, 2015, 2016 Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <limits>
#include <memory>
#include <vector>

#include "gtest/gtest.h"
#include "robot_localization/ekf.hpp"
#include "robot_localization/filter_base.hpp"
#include "robot_localization/filter_common.hpp"
#include "robot_localization/ros_filter.hpp"
#include "robot_localization/ros_filter_types.hpp"

using robot_localization::Ekf;
using robot_localization::RosEkf;
using robot_localization::STATE_SIZE;

class PoseTimeTestFilter : public RosEkf
{
public:
  explicit PoseTimeTestFilter(const rclcpp::NodeOptions & options) : RosEkf(options) {}
  using RosEkf::preparePose;

  void addTransform(const std::string & parent, const std::string & child,
    uint32_t nanosec, double x)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.header.stamp.sec = 1791000000;
    transform.header.stamp.nanosec = nanosec;
    transform.transform.translation.x = x;
    transform.transform.rotation.w = 1.0;
    ASSERT_TRUE(tf_buffer_->setTransform(transform, "test", false));
  }
};

TEST(EkfTest, PoseUsesMeasurementTimeForBothFrameTransforms) {
  // At .500 s the interpolated transform is x=5; the latest is x=9.
  // Looking up seconds as nanoseconds silently fell back to x=9 before the fix.
  for (bool source_transform : {false, true}) {
    rclcpp::NodeOptions options;
    options.arguments({"ekf_pose_time_test"});
    auto filter = std::make_shared<PoseTimeTestFilter>(options);
    filter->initialize();
    const std::string parent = source_transform ? "sensor_body" : "odom";
    const std::string child = source_transform ? "base_link" : "sensor_world";
    filter->addTransform(parent, child, 125000000, 1.0);
    filter->addTransform(parent, child, 875000000, 9.0);
    auto message = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    message->header.frame_id = source_transform ? "odom" : "sensor_world";
    message->header.stamp.sec = 1791000000;
    message->header.stamp.nanosec = 500000000;
    message->pose.pose.orientation.w = 1.0;
    for (int i = 0; i < 6; ++i) {
      message->pose.covariance[i * 7] = 0.01;
    }
    std::vector<bool> update(STATE_SIZE, false);
    update[0] = true;
    Eigen::VectorXd measurement = Eigen::VectorXd::Zero(STATE_SIZE);
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(STATE_SIZE, STATE_SIZE);
    ASSERT_TRUE(filter->preparePose(message, "time_test", "odom",
      source_transform ? "sensor_body" : "base_link", false, false, false,
      update, measurement, covariance));
    EXPECT_NEAR(measurement[0], 5.0, 1e-9);
  }
}

TEST(EkfTest, Measurements) {
  rclcpp::NodeOptions options;
  options.arguments({"ekf_filter_node"});
  std::shared_ptr<robot_localization::RosEkf> filter =
    std::make_shared<robot_localization::RosEkf>(options);
  filter->initialize();

  // create the instance of the class and pass parameters
  Eigen::MatrixXd initialCovar(15, 15);

  initialCovar.setIdentity();
  initialCovar *= 0.5;

  filter->getFilter().setEstimateErrorCovariance(initialCovar);

  Eigen::VectorXd measurement(STATE_SIZE);
  measurement.setIdentity();

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurement[i] = i * 0.01 * STATE_SIZE;
  }
  Eigen::MatrixXd measurementCovariance(STATE_SIZE, STATE_SIZE);
  measurementCovariance.setIdentity();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }
  std::vector<bool> updateVector(STATE_SIZE, true);

  // Ensure that measurements are being placed in the queue correctly
  rclcpp::Time time1(1000);
  filter->robot_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time1);

  filter->robot_localization::RosEkf::integrateMeasurements(rclcpp::Time(1001));

  EXPECT_EQ(filter->getFilter().getState(), measurement);
  EXPECT_EQ(
    filter->getFilter().getEstimateErrorCovariance(),
    measurementCovariance);

  filter->getFilter().setEstimateErrorCovariance(initialCovar);

  // Now fuse another measurement and check the output.
  // We know what the filter's state should be when
  // this is complete, so we'll check the difference and
  // make sure it's suitably small.
  Eigen::VectorXd measurement2 = measurement;

  measurement2 *= 2.0;

  for (size_t i = 0; i < STATE_SIZE; ++i) {
    measurementCovariance(i, i) = 1e-9;
  }

  rclcpp::Time time2(1002);

  filter->robot_localization::RosEkf::enqueueMeasurement(
    "odom0", measurement2, measurementCovariance, updateVector,
    std::numeric_limits<double>::max(), time2);

  filter->robot_localization::RosEkf::integrateMeasurements(rclcpp::Time(1003));

  measurement = measurement2.eval() - filter->getFilter().getState();
  for (size_t i = 0; i < STATE_SIZE; ++i) {
    EXPECT_LT(::fabs(measurement[i]), 0.001);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return ret;
}
