//---------------------------------------------------------------------------------------------------------------------
//  ROS wrapper MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 - Marco Montes Grova (a.k.a. mgrova)  marrcogrova@gmail.com
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

#include <flow/flow.h>
#include <ros/ros.h>

#include <mico/ros_wrapper/flow/BlockROSSubscriber.h>
#include <mico/ros_wrapper/flow/BlockROSPublisher.h>

namespace ros_wrapper{

    extern "C" flow::PluginNodeCreator* factory(){

            std::map<std::string, std::string> remaps;
            remaps["__name"] = "ros_mplugin";

            ros::init(remaps, "ros_mplugin", (uint32_t)0);
            ros::AsyncSpinner spinner(4);
            spinner.start();

            flow::PluginNodeCreator *creator = new flow::PluginNodeCreator;

            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSSubscriberPoseStamped> >(); }, "ROS");
            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSSubscriberCloud      > >(); }, "ROS");
            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSSubscriberImu        > >(); }, "ROS");
            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSSubscriberGPS        > >(); }, "ROS");
            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSSubscriberImage      > >(); }, "ROS");

            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSPublisherPoseStamped > >(); }, "ROS");
            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSPublisherPointCloud  > >(); }, "ROS");
            creator->registerNodeCreator([](){ return std::make_unique<flow::FlowVisualBlock<BlockROSPublisherImage       > >(); }, "ROS");

            return creator;
        }
    
}