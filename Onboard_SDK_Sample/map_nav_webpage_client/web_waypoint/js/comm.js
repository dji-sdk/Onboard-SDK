//var seq_string = randomString(7);
//var mission_info = {
//    "SEQ":  		"null",
//    "OPERATION": 	"UploadWayLineMission",
//    "VALUE": 	{
//        "LENGTH": 			me.mission_info_struct.mission_length,
//        "VEL_CMD_RANGE": 	me.mission_info_struct.vel_cmd_range,
//        "IDLE_VEL": 		me.mission_info_struct.idle_vel,
//        "ACTION_ON_FINISH": 	me.mission_info_struct.action_on_finish,
//        "MISSION_EXEC_NUM": 	me.mission_info_struct.mission_exec_num,
//        "YAW_MODE": 		me.mission_info_struct.yaw_mode,
//        "TRACE_MODE": 		me.mission_info_struct.trace_mode,
//        "ACTION_ON_RC_LOST": me.mission_info_struct.action_on_rc_lost,
//        "GIMBAL_PITCH_MODE": me.mission_info_struct.gimbal_pitch_mode,
//        "HP_LATI": 			me.mission_info_struct.hp_lati,
//        "HP_LONTI": 		me.mission_info_struct.hp_lonti,
//        "HP_ALTI": 			me.mission_info_struct.hp_alti
//    }
//};

/*****
 *
 * define communication protocols
 */

var KEY_SEQ = "SEQ";
var KEY_OP = "OPERATION";
var KEY_VAL = "VALUE";
var eventSeq = null;
var eventValue = null;
var eventResult = null;
var eventErrmsg = null;

var set_navigation_mode = {
    "SEQ":       "null",
    "OPERATION": "SetNavigationMode",
    "VALUE":     {
        "NAVIGATION_MODE":   1         // 1: Enter    2: Exit
    }
};

var upload_mission_info = {
    "SEQ":  		"null",
    "OPERATION": 	"UploadWayLineMission",
    "VALUE": 	{
        "LENGTH": 			0,//# of points
        "VEL_CMD_RANGE": 	10,//velocity range from RC
        "IDLE_VEL": 		10,
        "ACTION_ON_FINISH": 0,
        "MISSION_EXEC_NUM": 0, //Mission repeat times
        "YAW_MODE": 		0,
        "TRACE_MODE": 		0,
        "ACTION_ON_RC_LOST": 0,
        "GIMBAL_PITCH_MODE": 0,
        "HP_LATI": 			22.540091*Math.PI/180, //The home info should be set as the one in drone MCU
        "HP_LONTI": 		113.946593*Math.PI/180,
        "HP_ALTI": 			100
    }
};

var download_waypoint_mission_info = {
    "SEQ":       "null",
    "OPERATION": "DownloadWayLineMission"
};

var upload_waypoint = {
    "SEQ":       "null",
    "OPERATION": "UploadWayPoint",
    "VALUE":     {
        "INDEX": 			0,
        "LATI": 			0,
        "LONTI":			0,
        "ALTI": 			120,
        "DAMPING_DIS": 		180,
        "TGT_YAW": 			30,
        "TGT_GIMBAL_PITCH": -120,
        "TURN_MODE": 		0,
        "HAS_ACTION":  		0,
        "ACTION_TIME_LIMIT":50,
        "ACTION": {
            "ACTION_NUM": 	15,
            "ACTION_RPT": 	15,
            "COMMAND_LIST":	{
                "WP_ACTION_STAY": 200,
                "WP_ACTION_SIMPLE_SHOT": 1,
                "WP_ACTION_VIDEO_START": 1,
                "WP_ACTION_VIDEO_STOP": 1,
                "WP_ACTION_CRAFT_YAW": 200,
                "WP_ACTION_GIMBAL_YAW": 200,
                "WP_ACTION_GIMBAL_PITCH": 200
            }
        }

    }
};

var download_waypoint = {
    "SEQ":       "null",
    "OPERATION": "DownloadWayPoint",
    "VALUE":{
        "INDEX": 2
    }
};

var startstop_waypoint_mission = {
    "SEQ":       "null",
    "OPERATION": "StartOrCancelWayLineMission",
    "VALUE":  {
       "GO_STOP": 0    // 0: go    1: stop
    }
};

var pauseresume_waypoint_mission = {
    "SEQ":       "null",
    "OPERATION": "PauseOrContinueWayLineMission",
    "VALUE":  {
      "PAUSE": 0    // 0: pause    1: continue
    }
};

var set_waypoint_idle_vel = {
    "SEQ":       "null",
    "OPERATION": "SetWayLineFlightIdelValue",
    "VALUE":  {
        "IDLE_VEL": 0
    }
};

var get_waypoint_idle_vel = {
    "SEQ":       "null",
    "OPERATION": "GetWayLineFlightIdelValue",
};


var navigation_runnning_status_info = {
    "EVENT": "data_update",
    "OPERATION": "PushNavigationStatusInfo",
    "VALUE": {
        "MISSION_TYPE": 0,
        "TARGET_WAYPOINT": 10,
        "CURR_STATE": 0,
        "ERROR_NOTIFICATION": 0
    }
};

var navigation_mission_status_info = {
    "EVENT": "data_update",
    "OPERATION": "PushNavigationStatusInfo",
    "VALUE": {
        "MISSION_TYPE": 1,
        "LAST_MISSION_TYPE": 10,
        "IS_BROKEN": 0,
        "REASON": 0
    }
};

var navigation_runnning_event_info = {
    "EVENT": "data_update",
    "OPERATION": "PushNavigationEventInfo",
    "VALUE": {
        "INCIDENT_TYPE": 0,
        "WAYPOINT_INDEX": 0,
        "CURR_STATE": 0
    }
};

var navigation_mission_event_info = {
    "EVENT": "data_update",
    "OPERATION": "PushNavigationEventInfo",
    "VALUE": {
        "INCIDENT_TYPE": 1,
        "REPEAT": 0
    }
};

//update waypoint data
function updateWPData(seq, index, lati, longi, alti, damping_dis, tgt_yaw, tgt_gimbal_pitch, turn_mode, has_action, action_time_limit, action_rpt, action_list, action_param) {

    var newWaypoint = upload_waypoint;

    // need a evaluation for valid input parameters
    newWaypoint.SEQ = seq;
    newWaypoint.VALUE.INDEX = index;
    newWaypoint.VALUE.LATI = lati*Math.PI/180;
    newWaypoint.VALUE.LONTI = longi*Math.PI/180;
    newWaypoint.VALUE.ALTI = alti;

    //null stands for defualt value
    newWaypoint.VALUE.DAMPING_DIS = damping_dis == null?upload_waypoint.VALUE.DAMPING_DIS:damping_dis;
    newWaypoint.VALUE.TGT_YAW = tgt_yaw == null? upload_waypoint.VALUE.TGT_YAW : tgt_yaw;
    newWaypoint.VALUE.TGT_GIMBAL_PITCH = tgt_gimbal_pitch == null? upload_waypoint.VALUE.TGT_GIMBAL_PITCH : tgt_gimbal_pitch;
    newWaypoint.VALUE.TURN_MODE = turn_mode == null? upload_waypoint.VALUE.TURN_MODE : turn_mode;
    newWaypoint.VALUE.HAS_ACTION = has_action == null? upload_waypoint.VALUE.HAS_ACTION : has_action;
    newWaypoint.VALUE.ACTION_TIME_LIMIT = action_time_limit == null? upload_waypoint.VALUE.ACTION_TIME_LIMIT : action_time_limit;

    if(newWaypoint.VALUE.HAS_ACTION != 0) {
        newWaypoint.VALUE.ACTION.ACTION_NUM = action_list.length;
        newWaypoint.VALUE.ACTION.ACTION_RPT = action_rpt;

        //Action_list is generated ouside, a better way.
        //newWaypoint.VALUE.COMMAND_LIST = action_list;
    }

    return newWaypoint;
}

//function uploadMissionInfo(mission_info) {
//    $.ajax({
//        url: 'ws://localhost:19870/controller/navigation',
//        type: 'POST',
//        data: JSON.stringify(mission_info),
//        contentType: 'application/json; charset=utf-8',
//        dataType: 'json',
//        async: true,
//        success: function(msg) {
//            console.log(msg);
//        }
//    });
//}
var COMM_STANDBY  = 0,
    COMM_SENDING  = 1,
    COMM_FINISHED = 2;

var home_lat = 0;
var home_lon = 0;
var home_hei = 0;

var isExecuted = false;


function Communicator(socket) {

    //rosbridge config
    this.ros = new ROSLIB.Ros({
        url : socket.url
    });

    this.ros.on('connection', function(){
        console.log('Connected to websocket server.');
        $( '<div>Connected to websocket server.</div>' ).appendTo("#monitor");
    });
    this.ros.on('error', function(){
        console.log('Error connecting to websocket server: ', error);
        $( '<div>Error connecting to websocket server: '+error+'</div>' ).appendTo("#monitor");
    });
    this.ros.on('close', function(){
        console.log('Connection is closed!.');
        $( '<div>Connection is closed!</div>' ).appendTo("#monitor");
    });

    this.tid = 0;

    //action client
    this.web_wp_client = new ROSLIB.ActionClient({
        ros : this.ros,
        serverName : '/DJI_ROS/web_waypoint_receive_action',
        actionName : 'dji_ros/web_waypoint_receiveAction'
    });;

    //publisher
    this.ctrlTopic = new ROSLIB.Topic({
        ros : this.ros,
        name : '/DJI_ROS/map_nav_srv/ctrl',
        messageType : 'std_msgs/Bool'
    });
    this.cmdTopic = new ROSLIB.Topic({
        ros : this.ros,
        name : '/DJI_ROS/map_nav_srv/cmd',
        messageType : 'dji_ros/map_nav_srv_cmd'
    });

    //subscriber
    this.rosListener = new ROSLIB.Topic({
        ros : this.ros,
        name : '/DJI_ROS/global_position',
        messageType : 'dji_ros/global_position'
    });
    this.rosListener.subscribe(function(msg) {
        home_lat = msg.latitude / Math.PI * 180;
        home_lon = msg.longitude / Math.PI * 180;
        home_hei = msg.height;
    });
    /*home_lat = 22.542494;
    home_lon = 113.958004;*/
    //this.getGlobalPosition();

}
/*
Communicator.prototype.getGlobalPosition = function() {
    //subscriber
    this.rosListener = new ROSLIB.Topic({
        ros : this.ros,
        name : '/DJI_ROS/global_position',
        messageType : 'dji_ros/global_position'
    });
    this.rosListener.subscribe(function(msg) {
        home_lat = msg.latitude / Math.PI * 180;
        home_lon = msg.longitude / Math.PI * 180;
        this.unsubscribe()
    });
}
*/
//Actually, only the upload waypoint list need an extra function to check whether done or not.
//As for others, they all can be handled in `onmessage`
//Need to refactor code in the same style, this `while` way, or the `onmessage` way.
//But need to test the delay problem in case of traffic jam and SEQ mess
//since ACK has no OPERATION
Communicator.prototype.checkDone = function(sequence) {
    var me = this;
    /*
    0x00 --> success
    0x01 --> not success
    0x10 --> still waiting
    0x11 --> reserve
     */
    console.log (sequence + "," + eventSeq + "," + eventResult);
    if((sequence == eventSeq) && (eventResult == "SUCCESS")) {
        if (eventValue == null)
            //ACK
            return 0x00;
        else
            //cmd for get_xxx
            return eventValue;
    }
    else if ((sequence == eventSeq) && (eventResult != "SUCCESS")) {
        //throw errors
        //or some other way
        var errmsg = eventResult + "," + eventErrmsg;
        me.resetCheckParam();
        alert("Command Failed with Error: " + errmsg);
        throw("Command Failed with Error: " + errmsg);
    }
    else
        return 0x10;
};

Communicator.prototype.resetCheckParam = function() {
    eventSeq = null;
    eventValue = null;
    eventResult = null;
    eventErrmsg = null;
};

Communicator.prototype.setNavigationMode = function() {

    var _msg = new ROSLIB.Message({
        data : true
    });

    console.log('Request to obtain control');
    this.ctrlTopic.publish(_msg);

};

Communicator.prototype.stopNavigationMode = function() {

    var _msg = new ROSLIB.Message({
        data : false
    });

    console.log('Release control');
    this.ctrlTopic.publish(_msg);

};

Communicator.prototype.uploadWayline = function() {

    isExecuted = false;

    if (my_planner.markerList.length < 2){
        alert("Please set 2 waypoints at least!");
        return;
    }

    // rosbridge config
    this.tid = new Date().getTime();
    var _msg = new ROSLIB.Message({
        cmdCode : "n".charCodeAt(0),
        tid : this.tid
    });
    console.log('Upload new waypointLine');
    this.cmdTopic.publish(_msg);

    var _waypointList = new Array();
    for(i = 0; i < my_planner.markerList.length; i++) {
        var _wp = new ROSLIB.Message({
            latitude : my_planner.markerList[i][1].point.lat,
            longitude : my_planner.markerList[i][1].point.lng,
            altitude : my_planner.markerList[i][2].alti,
            heading : 0,
            staytime : 0
        });
        //since waypoint_navigation_action has been updated, it is no need to do such judgement
        /*if(i == 0 && Math.abs(my_planner.markerList[0][2].alti - home_hei) < 0.15
            && Math.abs(my_planner.markerList[0][1].point.lat - home_lat) < 0.00001 
            && Math.abs(my_planner.markerList[0][1].point.lng - home_lon) < 0.00001
        )
            continue;*/
        _waypointList.push(_wp);
    }
    var goal_waypointList = {
        waypointList : _waypointList
    };
    var goal = new ROSLIB.Goal({
        actionClient : this.web_wp_client,
        goalMessage : {
            waypointList : goal_waypointList,
            tid : this.tid
        }
    });

    goal.on('feedback', function(feedback) {
        //console.log('Feedback: current stage ' + feedback.stage);
        var stageMsg = '';
        switch(feedback.stage) {
            case 0:
                stageMsg = 'waiting for waypointList'; break;
            case 1:
                stageMsg = 'waiting for start'; break;
            case 2:
                stageMsg = 'in progress'; break;
            case 3:
                stageMsg = 'paused'; break;
            case 4:
                stageMsg = 'canceled'; break;
        }

        var str = '<div>Feedback: current stage ' + feedback.stage + ' - '
            + stageMsg + '</div>'
            + '<div>Latitude progress: ' + feedback.latitude_progress + '%</div>'
            + '<div>Longitude progress: ' + feedback.longitude_progress + '%</div>'
            + '<div>Altitude progress: ' + feedback.altitude_progress + '%</div>'
            + '<div>Index progress: ' + feedback.index_progress + '</div>';
        $("#state-update").empty();
        $( str ).appendTo("#state-update");
    });

    goal.on('result', function(result) {
        if(result.result) {
            console.log('Execution Succeed!');
            alert('Execution Succeed!');
        } else {
            console.log('Last Execution Fail!');
            if(isExecuted) alert('Last Execution Fail!');
        }
    });

    goal.send();

};

Communicator.prototype.downloadWayline= function() {

    var sequence =  "downloadWL";
    var downloadWayLine = download_waypoint_mission_info;

    downloadWayLine.SEQ = sequence;
    this.socket.send(JSON.stringify(downloadWayLine));
    this.waitingACK(sequence);
};

Communicator.prototype.uploadWaypoint = function(i) {
    var me = this;

    if(i >= my_planner.markerList.length)
        return;

    sequence = "uploadWP"+i;
    var waypoint = updateWPData(sequence,i,my_planner.markerList[i][1].point.lat,my_planner.markerList[i][1].point.lng,my_planner.markerList[i][2].alti,my_planner.markerList[i][2].damping_dis,my_planner.markerList[i][2].tgt_yaw,my_planner.markerList[i][2].tgt_gimbal_pitch,my_planner.markerList[i][2].turn_mode,my_planner.markerList[i][2].turn_mode,my_planner.markerList[i][2].has_action,my_planner.markerList[i][2].action_time_limit,my_planner.markerList[i][2].action_rpt,my_planner.markerList[i][2].cmd_list,my_planner.markerList[i][2].cmd_param);
    this.socket.send(JSON.stringify(waypoint));

    waitWPUploadResult();


    function waitWPUploadResult() {
        var doneFlag = me.checkDone(sequence);
        setTimeout(
            function() {
                if (doneFlag == 0x10){
                    console.log("Still Running");
                    waitWPUploadResult();
                }
                //The success will return successIndex, instead of 0x00 flag
                else{
                    console.log("Success");
                    me.uploadWaypoint(i+1);
                }
            }
        ,5);
    }

};

Communicator.prototype.downloadWaypoint = function(index) {
    var sequence = "downloadWP";
    var downloadWP = download_waypoint;
    downloadWP.SEQ = sequence;
    downloadWP.VALUE.INDEX = index;

    this.socket.send(JSON.stringify(downloadWP));
    this.waitingACK(sequence);
};

Communicator.prototype.startWayline = function() {

    isExecuted = true;

    var _msg = new ROSLIB.Message({
        cmdCode : "s".charCodeAt(0),
        tid : this.tid
    });

    console.log('Start task with tid ' + this.tid);
    this.cmdTopic.publish(_msg);

};

Communicator.prototype.cancelWayline = function() {

    //The function is not defined
    //this.web_wp_client.cancel();

    var _msg = new ROSLIB.Message({
        cmdCode : "c".charCodeAt(0),
        tid : this.tid
    });

    console.log('Cancel current task');
    this.cmdTopic.publish(_msg);

};

//TODO: have't implemented this function in ROS yet
Communicator.prototype.pauseWayline = function() {

    var _msg = new ROSLIB.Message({
        cmdCode : "p".charCodeAt(0),
        tid : this.tid
    });

    console.log('Pause task with tid ' + this.tid);
    this.cmdTopic.publish(_msg);

};

//TODO: have't implemented this function in ROS yet
Communicator.prototype.continueWayline = function() {

    var _msg = new ROSLIB.Message({
        cmdCode : "r".charCodeAt(0),
        tid : this.tid
    });

    console.log('Resume task with tid ' + this.tid);
    this.cmdTopic.publish(_msg);

};

Communicator.prototype.setWaylineIdelValue = function(idelValue) {

    var sequence = "setIdelValue";
    var setIdelValue = set_waypoint_idle_vel;

    setIdelValue.SEQ = sequence;
    setIdelValue.VALUE.IDLE_VEL= idelValue;

    this.socket.send(JSON.stringify(setIdelValue));
    this.waitingACK(sequence);

};

Communicator.prototype.getWaylineIdelValue = function() {

    var sequence = "getIdelValue";
    var getIdelValue = get_waypoint_idle_vel;

    getIdelValue.SEQ = sequence;

    this.socket.send(JSON.stringify(getIdelValue));
    this.waitingACK(sequence);

};

