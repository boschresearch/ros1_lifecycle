//
// ROS1 Lifecycle - A library implementing the ROS2 lifecycle for ROS1
//
// Copyright 2016,2017 Robert Bosch GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//

#include <gtest/gtest.h>

#include <boost/thread.hpp>
#include <boost/shared_ptr.hpp>

#include "lifecycle/managed_node.h"
#include "lifecycle/client.h"
#include "lifecycle/lifecycle_model.h"

using namespace ros;
using namespace ros::lifecycle;
using namespace lifecycle_msgs;
using namespace actionlib;

int argc_;
char** argv_;
#define INIT ros::init(argc_, argv_, "test_lifecycle_client"); ros::NodeHandle nh("~"); nh.setParam(PARAM_LIFECYCLE_MANAGEMENT, true);
#define LIFECYCLE_SERVER "test_lifecycle_client"
#define SLEEP_TIME 3.0

bool Result = false;
bool Executed = false;

namespace {

class LmNode: public ManagedNode
{
    public:
        LmNode(const ros::NodeHandle& nh): ManagedNode(nh) { };
    
    protected:
        bool onActivate() { return true;};
        bool onCleanup() { return true; };
        bool onDeactivate() { return true; };
        bool onError(const std::exception&) {return true;};
};

void spinThread()
{
  ros::spin();
}

void cb_func(bool result){
    Executed = true;
    Result = result;
}
}

TEST(Lifecycle_Client, test_apiexception)
{
    INIT
    boost::thread spin_thread(&spinThread);
    
    LifecycleClient lm_client(nh, LIFECYCLE_SERVER);
    
    Result = false;
    Executed = false;
    EXPECT_THROW(lm_client.goToState(UNCONFIGURED, cb_func), LifecycleAPIException);

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}

TEST(Lifecycle_Client, test_unconfigured)
{
    INIT
    boost::thread spin_thread(&spinThread);
    LmNode test_node(nh);    
    
    LifecycleClient lm_client(nh, LIFECYCLE_SERVER);
    
    Result = false;
    Executed = false;
    lm_client.goToState(UNCONFIGURED, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}
    
TEST(Lifecycle_Client, test_inactive)
{
    INIT
    boost::thread spin_thread(&spinThread);
    LmNode test_node(nh);    
    
    LifecycleClient lm_client(nh, LIFECYCLE_SERVER);
    
    Result = false;
    Executed = false;
    lm_client.goToState(INACTIVE, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);
    
    //Attempt 2nd time
    Result = false;
    Executed = false;
    lm_client.goToState(UNCONFIGURED, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);
    
    Result = false;
    Executed = false;
    lm_client.goToState(INACTIVE, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}
    
TEST(Lifecycle_Client, test_active)
{
    INIT
    boost::thread spin_thread(&spinThread);
    LmNode test_node(nh);    
    
    LifecycleClient lm_client(nh, LIFECYCLE_SERVER);
    
    Result = false;
    Executed = false;
    lm_client.goToState(ACTIVE, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);
    
    //Check from inactive to active
    Result = false;
    Executed = false;
    lm_client.goToState(INACTIVE, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);
    
    Result = false;
    Executed = false;
    lm_client.goToState(ACTIVE, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}
    
TEST(Lifecycle_Client, test_finalized)
{
    INIT
    boost::thread spin_thread(&spinThread);
    LmNode test_node(nh);    
    
    LifecycleClient lm_client(nh, LIFECYCLE_SERVER);
    
    Result = false;
    Executed = false;
    lm_client.goToState(FINALIZED, cb_func);
    ros::Duration(SLEEP_TIME).sleep();
    ASSERT_TRUE(Executed);
    ASSERT_TRUE(Result);

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}
    
int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    argc_ = argc;
    argv_ = argv;
    return RUN_ALL_TESTS();
}
