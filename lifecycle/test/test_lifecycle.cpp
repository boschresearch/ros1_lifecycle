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

#include <actionlib/client/simple_action_client.h>

#include "lifecycle/manager.h"
#include "lifecycle/managed_node.h"
#include "lifecycle_msgs/LifecycleGoal.h"
#include "lifecycle_msgs/Lifecycle.h"

using namespace ros;
using namespace ros::lifecycle;
using namespace lifecycle_msgs;
using namespace actionlib;

int argc_;
char** argv_;
#define INIT ros::init(argc_, argv_, "test_lifecycle"); ros::NodeHandle nh("~"); nh.setParam(PARAM_LIFECYCLE_MANAGEMENT, true);
#define LIFECYCLE_ACTION "test_lifecycle/lifecycle"

int Count = 0;

namespace {
bool returnFalse() {
    return false;
}
class ActiveNode: public ManagedNode
{
    public:
        ActiveNode(const ros::NodeHandle& nh): ManagedNode(nh) { };
    
    protected:
        bool onActivate() { return true;};
};
bool errorFalseCb(std::exception ex) {
    return false;
}
bool errorTrueCb(std::exception ex) {
    return true;
}
}

TEST(LifecycleManager, internal)
{
    INIT
    LifecycleManager lm(nh);
    lm.configure();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    lm.cleanup();
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());
    lm.configure();
    lm.activate();
    ASSERT_EQ(ACTIVE, lm.getCurrentState());
    lm.shutdown();
    ASSERT_EQ(FINALIZED, lm.getCurrentState());
}

typedef SimpleActionClient<LifecycleAction> LifecycleClient;
void check_goal(LifecycleManager& lm, LifecycleClient& client, int transition, State target)
{
    LifecycleGoal goal;

    goal.transition = transition;
    client.sendGoal(goal);
    ASSERT_TRUE(client.waitForResult(ros::Duration(0.5)));
    ASSERT_EQ(SimpleClientGoalState::SUCCEEDED, client.getState().state_);
    ASSERT_EQ(target, lm.getCurrentState());

}

void spinThread()
{
  ros::spin();
}

TEST(LifecycleManager, through_action_client)
{
    INIT
    LifecycleManager lm(nh);
    lm.start();
    
    SimpleActionClient<LifecycleAction> client(LIFECYCLE_ACTION);
    //A spin thread is required for the client!!!
    boost::thread spin_thread(&spinThread);
    ASSERT_TRUE(client.waitForServer(ros::Duration(2)));

    check_goal(lm, client, LifecycleGoal::EV_CONFIGURE, INACTIVE);
    check_goal(lm, client, LifecycleGoal::EV_CLEANUP, UNCONFIGURED);
    check_goal(lm, client, LifecycleGoal::EV_CONFIGURE, INACTIVE);
    check_goal(lm, client, LifecycleGoal::EV_ACTIVATE, ACTIVE);
    check_goal(lm, client, LifecycleGoal::EV_SHUTDOWN, FINALIZED);
    
    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}

TEST(LifecycleManager, test_transitions_for_unconfigured)
{
    INIT
    LifecycleManager lm(nh);
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());

    // make sure these are not accepted
    EXPECT_THROW(lm.activate(), IllegalTransitionException);
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());
    EXPECT_THROW(lm.deactivate(), IllegalTransitionException);
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());
    EXPECT_THROW(lm.cleanup(), IllegalTransitionException);
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());

    // now make sure all the valid ones are accepted
    ASSERT_TRUE(lm.configure());
    ASSERT_EQ(INACTIVE, lm.getCurrentState());

    lm.cleanup();
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());

    ASSERT_TRUE(lm.shutdown());
    ASSERT_EQ(FINALIZED, lm.getCurrentState());
}

TEST(LifecycleManager, test_transitions_for_inactive)
{
    INIT
    LifecycleManager lm(nh);
    lm.configure();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());

    // make sure these are not accepted
    EXPECT_THROW(lm.configure(), IllegalTransitionException);
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    EXPECT_THROW(lm.deactivate(), IllegalTransitionException);
    ASSERT_EQ(INACTIVE, lm.getCurrentState());

    // now make sure all the valid ones are accepted
    ASSERT_TRUE(lm.activate());
    ASSERT_EQ(ACTIVE, lm.getCurrentState());

    // go back
    lm.deactivate();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());

    ASSERT_TRUE(lm.cleanup());
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());

    // go back
    ASSERT_TRUE(lm.configure());
    ASSERT_EQ(INACTIVE, lm.getCurrentState());

    ASSERT_TRUE(lm.shutdown());
    ASSERT_EQ(FINALIZED, lm.getCurrentState());
}

TEST(LifecycleManager, test_transitions_for_active)
{
    INIT
    LifecycleManager lm(nh);
    lm.configure();
    lm.activate();
    ASSERT_EQ(ACTIVE, lm.getCurrentState());

    // make sure these are not accepted
    EXPECT_THROW(lm.configure(), IllegalTransitionException);
    ASSERT_EQ(ACTIVE, lm.getCurrentState());
    EXPECT_THROW(lm.activate(), IllegalTransitionException);
    ASSERT_EQ(ACTIVE, lm.getCurrentState());
    EXPECT_THROW(lm.cleanup(), IllegalTransitionException);
    ASSERT_EQ(ACTIVE, lm.getCurrentState());

    // now make sure all the valid ones are accepted
    ASSERT_TRUE(lm.deactivate());
    ASSERT_EQ(INACTIVE, lm.getCurrentState());

    // go back
    lm.activate();
    ASSERT_EQ(ACTIVE, lm.getCurrentState());

    ASSERT_TRUE(lm.shutdown());
    ASSERT_EQ(FINALIZED, lm.getCurrentState());
}

TEST(LifecycleManager, simulateFailureDuringConfiguring)
{
    INIT
    LifecycleManager lm(nh);
    lm.setTransitionCallback(CONFIGURE, returnFalse);

    ASSERT_FALSE(lm.configure());
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());
}

TEST(LifecycleManager, simulateFailureDuringActivating)
{
    INIT
    LifecycleManager lm(nh);
    lm.setTransitionCallback(ACTIVATE, returnFalse);
    ASSERT_TRUE(lm.configure());
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    ASSERT_FALSE(lm.activate());
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
}

void subCb(Lifecycle msg)
{
    int Expected_End_State[5]  = { INACTIVE, UNCONFIGURED, INACTIVE, ACTIVE, FINALIZED};
    int Expected_Transition[5]    = { CONFIGURE, CLEANUP, CONFIGURE, ACTIVATE, SHUTDOWN};
    int Expected_Result_Code[5]    = { SUCCESS, SUCCESS, SUCCESS, SUCCESS, SUCCESS};
    ASSERT_EQ(Expected_End_State[Count], msg.end_state);
    ASSERT_EQ(Expected_Transition[Count], msg.transition);
    ASSERT_EQ(Expected_Result_Code[Count], msg.result_code);
    Count++;
}

TEST(LifecycleManager, test_publishTransition)
{
    INIT
    LifecycleManager lm(nh);
    lm.start();
       
    ros::Subscriber sub = nh.subscribe(LIFECYCLE_STATE_TOPIC, 1000, subCb);
    boost::thread spin_thread(&spinThread);
    
    lm.configure();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    lm.cleanup();
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());
    lm.configure();
    lm.activate();
    ASSERT_EQ(ACTIVE, lm.getCurrentState());
    lm.shutdown();
    ASSERT_EQ(FINALIZED, lm.getCurrentState());
    ros::Duration(2).sleep();
    ASSERT_EQ(5, Count);
    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}

TEST(LifecycleManager, test_simulateErrorInActive)
{
    INIT
    LifecycleManager lm(nh);
    std::exception ex;
    lm.setErrorCb(errorTrueCb);
    
    lm.configure();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    lm.activate();
    ASSERT_EQ(ACTIVE, lm.getCurrentState());
    lm.raiseError(ex);
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());
    
    lm.setErrorCb(errorFalseCb);
    
    lm.configure();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    lm.activate();
    ASSERT_EQ(ACTIVE, lm.getCurrentState());
    lm.raiseError(ex);
    ASSERT_EQ(FINALIZED, lm.getCurrentState());
    
}

TEST(LifecycleManager, test_simulateErrorInInactive)
{
    INIT
    LifecycleManager lm(nh);
    std::exception ex;
    lm.setErrorCb(errorTrueCb);
    
    lm.configure();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    lm.raiseError(ex);
    ASSERT_EQ(UNCONFIGURED, lm.getCurrentState());
    
    lm.setErrorCb(errorFalseCb);
    
    lm.configure();
    ASSERT_EQ(INACTIVE, lm.getCurrentState());
    lm.raiseError(ex);
    ASSERT_EQ(FINALIZED, lm.getCurrentState());
    
}

void send_goal(LifecycleClient& client, int transition)
{
    LifecycleGoal goal;

    goal.transition = transition;
    client.sendGoal(goal);
    ASSERT_TRUE(client.waitForResult(ros::Duration(2)));
}

TEST(ManagedNode, test_defaultCleanup)
{
    INIT
    boost::thread spin_thread(&spinThread);
    ActiveNode test_node(nh);
    
    SimpleActionClient<LifecycleAction> client(LIFECYCLE_ACTION);
    ASSERT_TRUE(client.waitForServer(ros::Duration(2)));

    send_goal(client, LifecycleGoal::EV_CONFIGURE);
    ASSERT_EQ(INACTIVE, client.getResult()->end_state);
    
    send_goal(client, LifecycleGoal::EV_CLEANUP);
    ASSERT_EQ(FINALIZED, client.getResult()->end_state);

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}

TEST(ManagedNode, test_defaultDeactivate)
{
    INIT
    boost::thread spin_thread(&spinThread);
    ActiveNode test_node(nh);
    
    SimpleActionClient<LifecycleAction> client(LIFECYCLE_ACTION);
    ASSERT_TRUE(client.waitForServer(ros::Duration(2)));

    send_goal(client, LifecycleGoal::EV_CONFIGURE);
    ASSERT_EQ(INACTIVE, client.getResult()->end_state);
    
    send_goal(client, LifecycleGoal::EV_ACTIVATE);
    ASSERT_EQ(ACTIVE, client.getResult()->end_state);
    
    send_goal(client, LifecycleGoal::EV_DEACTIVATE);
    ASSERT_EQ(FINALIZED, client.getResult()->end_state);

    // shutdown the node and join the thread back before exiting
    ros::shutdown();
    spin_thread.join();
}
    
TEST(ManagedNode, test_defaultShutdown)
{
    INIT
    boost::thread spin_thread(&spinThread);
    ActiveNode test_node(nh);
    
    SimpleActionClient<LifecycleAction> client(LIFECYCLE_ACTION);
    ASSERT_TRUE(client.waitForServer(ros::Duration(2)));

    send_goal(client, LifecycleGoal::EV_CONFIGURE);
    ASSERT_EQ(INACTIVE, client.getResult()->end_state);
    
    send_goal(client, LifecycleGoal::EV_ACTIVATE);
    ASSERT_EQ(ACTIVE, client.getResult()->end_state);
    
    send_goal(client, LifecycleGoal::EV_SHUTDOWN);
    ASSERT_EQ(FINALIZED, client.getResult()->end_state);

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
