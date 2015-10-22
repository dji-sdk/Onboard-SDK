#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include "SDK.h"

using namespace actionlib;

typedef dji_ros::web_waypoint_receiveAction Action_t;
typedef dji_ros::web_waypoint_receiveGoal Goal_t;
typedef dji_ros::web_waypoint_receiveGoalConstPtr GoalConstPtr_t;
typedef dji_ros::web_waypoint_receiveFeedback Feedback_t;
typedef dji_ros::web_waypoint_receiveResult Result_t;

typedef dji_ros::waypoint_navigationAction WPAction_t;

SimpleActionClient<WPAction_t>* wpClientPtr_;
SimpleActionServer<Action_t>* asPtr_;
ros::ServiceClient* drone_ctrl_mgr_ptr;

uint8_t cmdCode_ = 0;
uint8_t stage_ = 0;
uint64_t tid_ = 0;
uint64_t cmdTid_ = 1;

uint8_t lat_p_; //latitude_progress
uint8_t lon_p_; //longitude_progress
uint8_t alt_p_; //altitude_progress
uint8_t idx_p_; //index_progress


void wp_feedbackCB(const dji_ros::waypoint_navigationFeedbackConstPtr& fb) {
    lat_p_ = fb->latitude_progress;
    lon_p_ = fb->longitude_progress;
    alt_p_ = fb->altitude_progress;
    idx_p_ = fb->index_progress;
}

void goalCB() {
    Feedback_t fb;
    Result_t rslt;

    cmdCode_ = 0; //eliminate effect of last task
    Goal_t newGoal = *( asPtr_->acceptNewGoal() );
    ROS_INFO_STREAM( "Received goal: \n" << newGoal << "\n" );

    //********** stage code **********
    //*  0: waiting for waypointList
    //*  1: waiting for start
    //*  2: in progress
    //*  3: paused
    //*  4: canceled
    //********************************

    //check last task stage
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
            //cmdCode_ = 0; //eliminate effect of next task
            ROS_INFO("Cancel task with tid %llu", tid_);
            stage_ = 4;
            fb.stage = stage_;
            asPtr_->publishFeedback(fb);
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
        } else {
            if(cmdCode_ == 'n') { //"n" for newer waypointLine arrived
                ROS_INFO("A latest task arrived.");
                ROS_INFO("Set task(if any) with tid %llu preemted", newGoal.tid);
                stage_ = 4;
                fb.stage = stage_;
                asPtr_->publishFeedback(fb);
            }
        }

        /*ROS_INFO_ONCE("Stage: %d", stage_);
        ROS_INFO_ONCE("CmdCode: %d", cmdCode_);
        ROS_INFO_ONCE("tid: %llu", tid_);*/

        bool isFinished; //flag for task result
        int cnt; //feedback count
        dji_ros::waypoint_navigationGoal wpGoal; //to call waypoint action
        switch(stage_) {
            case 0: //"0" for waiting for waypointList
                rslt.result = false;
                asPtr_->setAborted(rslt);
                ROS_INFO("The task is aborted for no waypointList received.");
                stage_ = 0;
                tid_ = 0;
                return;
            case 1: //"1" for waiting for start
            case 3: //"3" for paused
                continue;
            case 2: //"2" for in progress
                wpClientPtr_->waitForServer();
                wpGoal.waypointList = wpl;
                wpClientPtr_->sendGoal(wpGoal, 
                    SimpleActionClient<WPAction_t>::SimpleDoneCallback(), 
                    SimpleActionClient<WPAction_t>::SimpleActiveCallback(), 
                    &wp_feedbackCB
                );
                ROS_DEBUG("[DEBUG] The goal is sent!");

                //wait 1200*0.25s=300s until the task finishes
                cnt = 1200;
                while(ros::ok() && cnt--) {
                    fb.latitude_progress = lat_p_;
                    fb.longitude_progress = lon_p_;
                    fb.altitude_progress = alt_p_;
                    fb.index_progress = idx_p_;
                    asPtr_->publishFeedback(fb);
                    ROS_DEBUG_COND(cnt==1199, "[DEBUG] cmdCode_: %c", cmdCode_);
                    if(cmdCode_ == 'c') {
                        //cmdCode_ = 0; //eliminate effect of next task
                        rslt.result = false;
                        asPtr_->setAborted(rslt);
                        ROS_INFO("The task is canceled while executing.");
                        stage_ = 0;
                        tid_ = 0;
                        wpClientPtr_->cancelGoal();
                        return;
                    }
                    isFinished = wpClientPtr_->waitForResult(ros::Duration(0.25));
                }
                if(isFinished) {
                    ROS_INFO("Action finished: %s", 
                        wpClientPtr_->getState().toString().c_str()
                    );
                    rslt.result = true;
                    asPtr_->setSucceeded(rslt);
                } else {
                    ROS_INFO("The task cannot finish before the time out.");
                    rslt.result = false;
                    asPtr_->setAborted(rslt);
                }
                stage_ = 0;
                tid_ = 0;
                return;
            case 4: //"4" for canceled
                ROS_INFO("The task is canceled.");
                stage_ = 0;
                tid_ = 0;
                rslt.result = false;
                asPtr_->setPreempted(rslt);
                wpClientPtr_->cancelAllGoals();
                return;
        }
    }
}

//TODO: preemptCB
void preemptCB() {
    ROS_INFO("Hey! I got preempt!");
}

void cmdCB(const dji_ros::map_nav_srv_cmdConstPtr& msg) {
    ROS_INFO("Received command \"%c\" of tid %llu", msg->cmdCode, msg->tid);
    cmdCode_ = msg->cmdCode;
    cmdTid_ = msg->tid;
}

//TRUE for request control and FALSE for release control
void ctrlCB(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data)
        ROS_INFO("Request to obtain control");
    else
        ROS_INFO("Release control");
    dji_ros::control_manager srv_ctrl;

    srv_ctrl.request.control_ability = msg->data;
    drone_ctrl_mgr_ptr->call(srv_ctrl);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "map_nav_srv");
    ros::NodeHandle nh;

    //drone control manager
    ros::ServiceClient ctrl_mgr = nh.serviceClient<dji_ros::control_manager>(
        "DJI_ROS/obtain_release_control"
    );
    drone_ctrl_mgr_ptr = &ctrl_mgr;

    //waypoint_navigation action server
    wpClientPtr_ = new SimpleActionClient<WPAction_t>(
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
    ros::Subscriber sub1 = nh.subscribe("/DJI_ROS/map_nav_srv/cmd", 1, cmdCB);
    ros::Subscriber sub2 = nh.subscribe("/DJI_ROS/map_nav_srv/ctrl", 1, ctrlCB);

    asPtr_->registerGoalCallback(&goalCB);
    asPtr_->registerPreemptCallback(&preemptCB);
    asPtr_->start();

    //use multi thread to handle the subscribers.
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
