//---------------------------------------------------------------------------------------------------------------------
//  flow
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

#ifndef ROS_MPLUGIN_STREAMERS_H_
#define ROS_MPLUGIN_STREAMERS_H_

#include <mico/ros_wrapper/streamers/BlockROSSubscriber.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
// #include <dvs_msgs/EventArray.h>

namespace ros_mplugin{

	template<typename T_>
	struct TraitRosSubscriber{
		static std::string blockName_;
		static std::map<std::string, std::string> output_;
		static std::any conversion_(std::string _tag, const typename T_::ConstPtr &_msg);
		typedef T_ ROSType_;
	};

	typedef TraitRosSubscriber<geometry_msgs::PoseStamped> 	TraitPoseStamped;
	typedef TraitRosSubscriber<sensor_msgs::Imu> 			TraitImu;
	typedef TraitRosSubscriber<sensor_msgs::NavSatFix> 		TraitGPS;
	typedef TraitRosSubscriber<sensor_msgs::Image> 			TraitImage;
	typedef TraitRosSubscriber<sensor_msgs::PointCloud2> 	TraitCloud;
	// typedef TraitRosSubscriber<dvs_msgs::EventArray> 		TraitEvent;

	typedef BlockROSSubscriber< TraitPoseStamped > 			BlockROSSubscriberPoseStamped;
	typedef BlockROSSubscriber< TraitCloud       > 			BlockROSSubscriberCloud;
	typedef BlockROSSubscriber< TraitImu         > 			BlockROSSubscriberImu;
	typedef BlockROSSubscriber< TraitGPS         > 			BlockROSSubscriberGPS;
	typedef BlockROSSubscriber< TraitImage       > 			BlockROSSubscriberImage;			
	// typedef BlockROSSubscriber< TraitEvent       > 			BlockROSSubscriberEventArray;			
}

#endif