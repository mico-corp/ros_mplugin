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

#ifndef ROS_MPLUGIN_BLOCK_PUBLISHER_H_
#define ROS_MPLUGIN_BLOCK_PUBLISHER_H_

#include <ros/ros.h>
#include <flow/Block.h>
#include <flow/Outpipe.h>

namespace ros_mplugin{

    template<typename _Trait >
    class BlockROSPublisher : public flow::Block{
    public:
        std::string name() {return _Trait::blockName_; }

		BlockROSPublisher(){
            createPolicy(_Trait::input_);
            for (auto tag : _Trait::input_){
                registerCallback({tag.first}, 
                                    [&](flow::DataFlow _data){
                                        auto topicContent =std::any_cast<typename _Trait::ROSType_>(_Trait::conversion_(_data));
                                        pubROS_.publish(topicContent);
                                    }
                );
            }    
        };

        virtual bool configure(std::unordered_map<std::string, std::string> _params) override{
            std::string topicPublish = _params["topic"];
            pubROS_ = nh_.advertise< typename _Trait::ROSType_ >(topicPublish, 1 );
            return true;
        }
        std::vector<std::string> parameters() override {return {"topic"};}

    private:
		ros::NodeHandle nh_;
		ros::Publisher pubROS_;
    };
}


#endif
