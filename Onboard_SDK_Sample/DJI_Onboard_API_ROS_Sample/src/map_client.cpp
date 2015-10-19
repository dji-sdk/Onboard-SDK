#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include "SDK.h"

using namespace actionlib;

typedef dji_ros::web_waypoint_receiveAction Action_t;
typedef dji_ros::web_waypoint_receiveGoal Goal_t;
typedef dji_ros::web_waypoint_receiveGoalConstPtr GoalConstPtr_t;
typedef dji_ros::web_waypoint_receiveFeedback Feedback_t;
typedef dji_ros::web_waypoint_receiveResult Result_t;

uint8_t cmdCode_ = 0;
uint8_t stage_ = 0;
uint64_t tid_ = 0;
uint64_t cmdTid_ = 1;

SimpleActionClient<dji_ros::waypoint_navigationAction>* wpClientPtr_;
SimpleActionServer<Action_t>* asPtr_;

void goalCB() {
    Feedback_t fb;
    Result_t rslt;

    Goal_t newGoal = *( asPtr_->acceptNewGoal() );
    ROS_INFO_STREAM( "Received goal: \n" << newGoal << "\n" );

    if(stage_ == 2) {
        rslt.result = false;
        asPtr_->setAborted(rslt, "Last task is in progress!");
        stage_ = 0;
        tid_ = 0;
        return;
    }
    if(stage_ == 3) {
        rslt.result = false;
        asPtr_->setAborted(rslt, "Last task is paused!");
        stage_ = 0;
        tid_ = 0;
        return;
    }

    tid_ = newGoal.tid;
    dji_ros::waypointList wpl = newGoal.waypointList;
    stage_ = 1;

    while(ros::ok()) {
        if(cmdCode_ == 'c') { //"c" for cancel
            ROS_INFO("Cancel task with tid %llu", tid_);
            stage_ = 4;
            fb.stage = stage_;
            asPtr_->publishFeedback(fb);
            stage_ = 0;
            break;
        } else if(tid_ == cmdTid_){
            if(cmdCode_ == 's' && stage_ == 1) { //"s" for start
                ROS_INFO("Start task with tid %llu", tid_);
                stage_ = 2;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
                continue;
            }
            if(cmdCode_ == 'p' && stage_ == 2) { //"p" for pause
                ROS_INFO("Pause task with tid %llu", tid_);
                stage_ = 3;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
                continue;
            }
            if(cmdCode_ == 'r' && stage_ == 3) { //"r" for resume
                ROS_INFO("Resume task with tid %llu", tid_);
                stage_ = 2;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
                continue;
            }
        }

        ROS_INFO_ONCE("Stage: %d", stage_);
        ROS_INFO_ONCE("CmdCode: %d", cmdCode_);
        ROS_INFO_ONCE("tid: %llu", tid_);

        bool isFinished;
        dji_ros::waypoint_navigationGoal wpGoal;
        switch(stage_) {
            case 0:
                rslt.result = false;
                asPtr_->setAborted(rslt, "No waypointList received!");
                stage_ = 0;
                tid_ = 0;
                return;
            case 1:
            case 3:
                continue;
            case 2:
                wpClientPtr_->waitForServer();
                wpGoal.waypointList = wpl;
                wpClientPtr_->sendGoal(wpGoal);

                isFinished = wpClientPtr_->waitForResult(ros::Duration(300));
                if(isFinished) {
                    ROS_INFO("Action finished: %s", 
                        wpClientPtr_->getState().toString().c_str()
                    );
                    rslt.result = true;
                    asPtr_->setSucceeded(rslt, "Succeed!");
                } else {
                    ROS_INFO("Action did not finish before the time out.");
                    rslt.result = false;
                    asPtr_->setAborted(rslt, 
                        "The task cannot finished before timeout!"
                    );
                }
                stage_ = 0;
                tid_ = 0;
                return;
            case 4:
                rslt.result = false;
                asPtr_->setAborted(rslt, "The task is canceled!");
                stage_ = 0;
                tid_ = 0;
                return;
        }
    }
}

//TODO: preemptCB
void preemptCB() {
}

void cmdCB(const dji_ros::map_client_cmdConstPtr& msg) {
    cmdCode_ = msg->cmdCode;
    cmdTid_ = msg->tid;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "map_client");
    ros::NodeHandle nh;

    //drone control manager
    ros::ServiceClient drone_ctrl_mgr = 
        nh.serviceClient<dji_ros::control_manager>(
            "DJI_ROS/obtain_release_control"
        );
    dji_ros::control_manager srv_ctrl;

    //waypoint_navigation action server
    wpClientPtr_ = new SimpleActionClient<dji_ros::waypoint_navigationAction>(
        "DJI_ROS/waypoint_navigation_action", 
        true
    );

    //web_waypoint_receive action server
    asPtr_ = new SimpleActionServer<Action_t>(
        nh, 
        "DJI_ROS/web_waypoint_receive_action", 
        false
    );

    //command subscribers
    ros::Subscriber sub1 = nh.subscribe("/DJI_ROS/map_client/cmd", 1, cmdCB);

    asPtr_->registerGoalCallback(&goalCB);
    asPtr_->registerPreemptCallback(&preemptCB);
    asPtr_->start();

    //TODO: get control. interface with js
    srv_ctrl.request.control_ability = 1;
    drone_ctrl_mgr.call(srv_ctrl);

    //TODO: release control. interface with js
    /*srv_ctrl.request.control_ability = 0;
    drone_ctrl_mgr.call(srv_ctrl);*/

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
