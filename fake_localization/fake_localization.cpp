/*********************************************************************
 * Software License Agreement (BSD License)
 * 
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

/** \author Ioan Sucan */

/**

  @mainpage

  @htmlinclude manifest.html

  @b odom_localization Takes in ground truth pose information for a robot
  base (e.g., from a simulator or motion capture system) and republishes
  it as if a localization system were in use.

  <hr>

  @section usage Usage
  @verbatim
  $ fake_localization
  @endverbatim

  <hr>

  @section topic ROS topics

  Subscribes to (name/type):
  - @b "base_pose_ground_truth" nav_msgs/Odometry : robot's odometric pose.  Only the position information is used (velocity is ignored).

  Publishes to (name / type):
  - @b "amcl_pose" geometry_msgs/PoseWithCovarianceStamped : robot's estimated pose in the map, with covariance
  - @b "particlecloud" geometry_msgs/PoseArray : fake set of poses being maintained by the filter (one paricle only).

  <hr>

  @section parameters ROS parameters

  - "~odom_frame_id" (string) : The odometry frame to be used, default: "odom"

 **/

#include <ros/ros.h>
#include <ros/time.h>

#include <nav_msgs/Odometry.h>
/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance

*/

#include <geometry_msgs/PoseArray.h>    // 一个 header + geometry_msgs/Pose[]数组
/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose[] poses
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/

#include <geometry_msgs/PoseWithCovarianceStamped.h>	// header + Pose + covariance
/*
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
*/

#include <angles/angles.h>

#include "ros/console.h"

#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>			// 存储已知的坐标系并提供一个 ROS 服务 “tf_frames”
#include <tf2_ros/transform_broadcaster.h>	// 共有函数只有 sendTransform() ,后面可以跟 StampedTransform 一个/数组, 在此文件中只发布一个
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include "message_filters/subscriber.h"


// "base_pose_ground_truth" 相当于已知了机器人的绝对位姿（base_link 相对于 map）,还要假装发布一个由 amcl 得到的定位结果	

class FakeOdomNode
{
  public:
    FakeOdomNode(void)		// 构造函数不需要任何参数
    {
      // 这就是两个普通的 ros::Publisher
      m_posePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,true);	// 通过仿真器报告的位姿
      m_particlecloudPub = m_nh.advertise<geometry_msgs::PoseArray>("particlecloud",1,true);	// 在rviz和nav_view中使用，来可视化机器人位姿的粒子云
      m_tfServer = new tf2_ros::TransformBroadcaster();		// 给 3 个指针指向的对象创建空间
      m_tfBuffer = new tf2_ros::Buffer();
      m_tfListener = new tf2_ros::TransformListener(*m_tfBuffer);	// 使用 tf2_ros::Buffer 初始化和构造 tf2_ros::TransformListener

      m_base_pos_received = false;

      ros::NodeHandle private_nh("~");
      private_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
      private_nh.param("base_frame_id", base_frame_id_, std::string("base_link")); 
      private_nh.param("global_frame_id", global_frame_id_, std::string("/map"));
      private_nh.param("delta_x", delta_x_, 0.0);		// 地图坐标系与仿真器坐标系原点在x轴方向的偏移。
      private_nh.param("delta_y", delta_y_, 0.0);		// 地图坐标系与仿真器坐标系原点在y轴方向的偏移。
      private_nh.param("delta_yaw", delta_yaw_, 0.0);		// 地图坐标系与仿真器坐标系原点在yaw偏航角的偏移。
      
      private_nh.param("transform_tolerance", transform_tolerance_, 0.1);      
      m_particleCloud.header.stamp = ros::Time::now();
      m_particleCloud.header.frame_id = global_frame_id_;
      m_particleCloud.poses.resize(1);
      ros::NodeHandle nh;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, -delta_yaw_);
      m_offsetTf = tf2::Transform(q, tf2::Vector3(-delta_x_, -delta_y_, 0.0));	// 这是一个逆变换, 从地图坐标系到仿真器坐标系

      stuff_sub_ = nh.subscribe("base_pose_ground_truth", 100, &FakeOdomNode::stuffFilter, this);	// 由模拟器公布的机器人的位置
      filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "", 100);	// message_filters::Subscriber 使用ROS主题作为其输入, 没有订阅的话题？？
      filter_ = new tf2_ros::MessageFilter<nav_msgs::Odometry>(*filter_sub_, *m_tfBuffer, base_frame_id_, 100, nh);	// base_frame_id_ = base_link 是指把订阅到的位姿从 odom 转换到 base_link 这个坐标系下
      filter_->registerCallback(boost::bind(&FakeOdomNode::update, this, _1));		// 回调一个 update 的函数

      // subscription to "2D Pose Estimate" from RViz:
      m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh, "initialpose", 1);	// message_filters::Subscriber 使用ROS主题作为其输入，允许使用 rviz 或 nav_view 等工具设置 fake_localization 的姿势，以提供与正在发布的地面真相源的自定义偏移。
      m_initPoseFilter = new tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*m_initPoseSub, *m_tfBuffer, global_frame_id_, 1, nh);		// 转到 map 坐标系下
      m_initPoseFilter->registerCallback(boost::bind(&FakeOdomNode::initPoseReceived, this, _1));	// 回调 initPoseReceived 函数
    }

    ~FakeOdomNode(void)			// 删除 3 个指针和为它们分配的空间
    {
      if (m_tfServer)
        delete m_tfServer; 
      if (m_tfListener)
        delete m_tfListener;
      if (m_tfBuffer)
        delete m_tfBuffer;
    }


  private:
    ros::NodeHandle m_nh;
    ros::Publisher m_posePub;	// m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,true)
    ros::Publisher m_particlecloudPub;	// m_nh.advertise<geometry_msgs::PoseArray>("particlecloud",1,true)
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseSub;
    tf2_ros::TransformBroadcaster       *m_tfServer;
    tf2_ros::TransformListener          *m_tfListener;
    tf2_ros::Buffer                     *m_tfBuffer;
    tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseFilter;
    tf2_ros::MessageFilter<nav_msgs::Odometry>* filter_;
    ros::Subscriber stuff_sub_; 			// nh.subscribe("base_pose_ground_truth", 100, &FakeOdomNode::stuffFilter, this)
    message_filters::Subscriber<nav_msgs::Odometry>* filter_sub_;

    double                         delta_x_, delta_y_, delta_yaw_;	// 0 , 0 , 0
    bool                           m_base_pos_received;
    double transform_tolerance_;			// 0.1

    nav_msgs::Odometry  m_basePosMsg;
    geometry_msgs::PoseArray      m_particleCloud;	// header + 大小为 1 的 Pose 数组，frame_id 是 "/map"
    geometry_msgs::PoseWithCovarianceStamped      m_currentPos;
    tf2::Transform m_offsetTf;				// = tf2::Transform(q, tf2::Vector3(-delta_x_, -delta_y_, 0.0))

    //parameter for what odom to use
    std::string odom_frame_id_;		// "odom"
    std::string base_frame_id_;		// "base_link"
    std::string global_frame_id_;	// "/map"

  public:
    // 是订阅 "base_pose_ground_truth" 之后的回调函数
    void stuffFilter(const nav_msgs::OdometryConstPtr& odom_msg){
      //we have to do this to force the message filter to wait for transforms
      //from odom_frame_id_ to base_frame_id_ to be available at time odom_msg.header.stamp     要使 从 odom 到 base_link 的坐标转换能够进行，才能回调这个函数
      //really, the base_pose_ground_truth should come in with no frame_id b/c it doesn't make sense
      boost::shared_ptr<nav_msgs::Odometry> stuff_msg(new nav_msgs::Odometry);
      *stuff_msg = *odom_msg;
      stuff_msg->header.frame_id = odom_frame_id_;	// 符合过滤器的要求，查找是否存在 odom 到 base_link 的坐标变换
      filter_->add(stuff_msg);		// 手动将消息添加到此筛选器中,参数类型是 const MConstPtr& ,如果消息（或队列中的任何其他消息）可立即转换，则会立即调用输出回调，可能多次调用
    }

    void update(const nav_msgs::OdometryConstPtr& message){	// filter_ 的回调，参数是从"base_pose_ground_truth" 订阅来的,把 frame_id 改成了 odom
      tf2::Transform txi;
      tf2::convert(message->pose.pose, txi);	// 由模拟器公布的机器人的位置
      txi = m_offsetTf * txi;		// map 中的 base_link ， map 中的机器人位置

      geometry_msgs::TransformStamped odom_to_map;
      try
      {
        geometry_msgs::TransformStamped txi_inv;
        txi_inv.header.frame_id = base_frame_id_;
        txi_inv.header.stamp = message->header.stamp;
        tf2::convert(txi.inverse(), txi_inv.transform);		// base_link 坐标系中的 map 原点

        m_tfBuffer->transform(txi_inv, odom_to_map, odom_frame_id_);	// base_link 坐标系转换到 odom坐标系下 ， odom 坐标系下的 map 位置
      }
      catch(tf2::TransformException &e)
      {
        ROS_ERROR("Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());		// 从 base_link 到 odom 转换失败
        return;
      }

      geometry_msgs::TransformStamped trans;
      trans.header.stamp = message->header.stamp + ros::Duration(transform_tolerance_);
      trans.header.frame_id = global_frame_id_;			// map
      trans.child_frame_id = message->header.frame_id;		// odom
      tf2::Transform odom_to_map_tf2;
      tf2::convert(odom_to_map.transform, odom_to_map_tf2);
      tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
      tf2::convert(odom_to_map_inv, trans.transform);
      m_tfServer->sendTransform(trans);			// trans 是 map 到 odom ， map 坐标系下的 odom 位置 ， 把 odom 添加到坐标树中

      tf2::Transform current;
      tf2::convert(message->pose.pose, current);	// 由模拟器公布的机器人的位置

      //also apply the offset to the pose
      current = m_offsetTf * current;			// map 中的机器人位置

      geometry_msgs::Transform current_msg;
      tf2::convert(current, current_msg);

      // Publish localized pose
      m_currentPos.header = message->header;
      m_currentPos.header.frame_id = global_frame_id_;		// frame_id = map
      tf2::convert(current_msg.rotation, m_currentPos.pose.pose.orientation);
      m_currentPos.pose.pose.position.x = current_msg.translation.x;
      m_currentPos.pose.pose.position.y = current_msg.translation.y;
      m_currentPos.pose.pose.position.z = current_msg.translation.z;
      m_posePub.publish(m_currentPos);				// 发布 map 中的机器人位置到 "amcl_pose"

      // The particle cloud is the current position. Quite convenient.
      m_particleCloud.header = m_currentPos.header;
      m_particleCloud.poses[0] = m_currentPos.pose.pose;
      m_particlecloudPub.publish(m_particleCloud);		// 发布一个粒子位置
    }

    void initPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg){		// msg 的 frame_id 一般为 map , 应该设置的是地图相对于仿真器的位姿
      tf2::Transform pose;
      tf2::convert(msg->pose.pose, pose);

      if (msg->header.frame_id != global_frame_id_){
        ROS_WARN("Frame ID of \"initialpose\" (%s) is different from the global frame %s", msg->header.frame_id.c_str(), global_frame_id_.c_str());
      }

      // set offset so that current pose is set to "initialpose"    
      geometry_msgs::TransformStamped baseInMap;
      try{
	// just get the latest
        baseInMap = m_tfBuffer->lookupTransform(base_frame_id_, global_frame_id_, msg->header.stamp);	// target:base_link ; source:map  查找 map 到 base_link 的坐标变换
      } catch(tf2::TransformException){
        ROS_WARN("Failed to lookup transform!");
        return;
      }

      tf2::Transform baseInMapTf2;
      tf2::convert(baseInMap.transform, baseInMapTf2);
      tf2::Transform delta = pose * baseInMapTf2;	// pose 是 initialPose
      m_offsetTf = delta * m_offsetTf;

    }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_localization");

  FakeOdomNode odom;

  ros::spin();

  return 0;
}
