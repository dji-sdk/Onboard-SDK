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
uint64_t id_ = 0;
Feedback_t feedback_;
Result_t result_;

void goalCB(SimpleActionServer<Action_t>& as, 
        SimpleActionClient<dji_ros::waypoint_navigationAction>& wpClient) {

    Goal_t newGoal = *as.acceptNewGoal();
    ROS_INFO_STREAM( "Received goal: \n" << newGoal << "\n" );

    if(stage_ == 2) {
        result_.result = false;
        as.setAborted(result_, "Last task is in progress!");
        return;
    }
    if(stage_ == 3) {
        result_.result = false;
        as.setAborted(result_, "Last task is paused!");
        return;
    }

    id_ = newGoal.id;
    dji_ros::waypointList wpl = newGoal.waypointList;
    stage_ = 1;

    while(ros::ok()) {
        //TODO: receive the cmd code
        if(cmdCode_ == 's' && stage_ == 1) { //"s" for start
            stage_ = 2;
            feedback_.stage = stage_;
            as.publishFeedback(feedback_);
            continue;
        }
        if(cmdCode_ == 'p' && stage_ == 2) { //"p" for pause
            stage_ = 3;
            feedback_.stage = stage_;
            as.publishFeedback(feedback_);
            continue;
        }
        if(cmdCode_ == 'r' && stage_ == 3) { //"r" for resume
            stage_ = 2;
            feedback_.stage = stage_;
            as.publishFeedback(feedback_);
            continue;
        }
        if(cmdCode_ == 'c') { //"c" for cancel
            stage_ = 4;
            feedback_.stage = stage_;
            as.publishFeedback(feedback_);
            stage_ = 0;
            break;
        }

        ROS_DEBUG("Stage: %d", stage_);
        ROS_DEBUG("CmdCode: %d", cmdCode_);
        ROS_DEBUG("id: %llu", id_);

        bool isFinished;
        dji_ros::waypoint_navigationGoal wpGoal;
        switch(stage_) {
            case 0:
                result_.result = false;
                as.setAborted(result_, "No waypointList received!");
                return;
            //case 1:
            case 3:
                continue;
            case 1:
            case 2:
                wpClient.waitForServer();
                wpGoal.waypointList = wpl;
                wpClient.sendGoal(wpGoal);

                isFinished = wpClient.waitForResult(ros::Duration(300));
                if(isFinished) {
                    ROS_INFO("Action finished: %s", 
                        wpClient.getState().toString().c_str()
                    );
                } else {
                    ROS_INFO("Action did not finish before the time out.");
                }
                return;
            case 4:
                result_.result = false;
                as.setAborted(result_, "The task is canceled!");
                return;
        }
    }
}

//TODO: preemptCB
void preemptCB(const SimpleActionServer<Action_t>& as) {
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
    SimpleActionClient<dji_ros::waypoint_navigationAction> wpClient(
        "DJI_ROS/waypoint_navigation_action", 
        true
    );

    //web_waypoint_receive action server
    SimpleActionServer<Action_t> as_(
        nh, 
        "DJI_ROS/web_waypoint_receive_action", 
        false
    );
    as_.registerGoalCallback( boost::bind(&goalCB, as_, wpClient) );
    as_.registerPreemptCallback( boost::bind(&preemptCB, as_) );
    as_.start();

    //TODO: get control. interface with js
    srv_ctrl.request.control_ability = 1;
    drone_ctrl_mgr.call(srv_ctrl);

    //TODO: release control. interface with js
    /*srv_ctrl.request.control_ability = 0;
    drone_ctrl_mgr.call(srv_ctrl);*/

    return 0;
}
