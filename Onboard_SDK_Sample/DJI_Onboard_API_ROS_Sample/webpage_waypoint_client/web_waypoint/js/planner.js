//some enum
if(typeof STATUS == "undefined"){
	var STATUS = {};
	STATUS.STANDBY      = 0;
	STATUS.FIRSTPENDING = 1;		//first point is jumping
	STATUS.FIRSTDONE    = 2;            //first point is done
	STATUS.FINISHED     = 3;
}

var ACTION_FINISH_HOVER = 0,
	ACTION_FINISH_GOHOME     = 1,
	ACTION_FINISH_LANDING    = 2,
	ACTION_FINISH_BACKFIRST  = 3,
	ACTION_FINISH_LOOP       = 4;

var YAW_MODE_AUTO = 0,
	YAW_MODE_LOCK  = 1,
	YAW_MODE_STICK = 2,
	YAW_MODE_WP    = 3,
	YAW_MODE_HP    = 4;

var TRACE_MODE_P2P  = 0,
	TRACE_MODE_TURN = 1;



// general functions
function randomString(len) {
　　len = len || 32;
　　var $chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnoprstuvwxyz0123456789';
　　var maxPos = $chars.length;
　　var pwd = '';
　　for (i = 0; i < len; i++) {
　　　　pwd += $chars.charAt(Math.floor(Math.random() * maxPos));
　　}
　　return pwd;
}

//class definitions
function missionInfo() {
	this.mission_length = 2;
	this.vel_cmd_range = 2;
	this.idle_vel = 1;
	this.action_on_finish =  ACTION_FINISH_GOHOME;
	this.mission_exec_num = 1;
	this.yaw_mode = YAW_MODE_AUTO;
	this.trace_mode = TRACE_MODE_P2P;
	this.action_on_rc_lost = 0;
	this.gimbal_pitch_mode = 0;
	this.hp_lati = 0;
	this.hp_lonti = 0;
	this.hp_alti = 0;
}
function wpOptions() {
	this.alti = 10;
	this.damping_dis = 10;
	this.tgt_yaw = 0;
	this.tgt_gimbal_pitch =0;
	this.turn_mode = 1;
	this.has_action = 0;
	this.action_time_limit = 0;

	this.action_num = 0;
	this.action_rpt = 0;
	this.cmd_list = [];
	this.cmd_param = [];
}
function Planner(bmap) {
	this.max_wp_num = 99;
	this.min_wp_distance = 3; // at least 3 meters

	this.map = bmap;
	this.markerList = new Array();
	this.lineList = new Array();

	// planner control variable
	this.planning_status = STATUS.STANDBY;
	this.curr_wp_num = 0;

	this.mission_info_struct = new missionInfo();

	//handler of click event listener
	this.clickListener = 0;
}

Planner.prototype.reset = function() {
	this.planning_status = STATUS.STANDBY;
	this.curr_wp_num = 0;
	this.markerList = new Array();
	this.lineList = new Array();
	this.map.clearOverlays();
	//remove listener to avoid duplicated confirm
        this.map.removeEventListener("click", this.clickListener);
	console.log("planner reset");
};

Planner.prototype.addFirstMarker = function(drone_home) {
	if (this.planning_status == STATUS.FINISHED)
		return;
	this.planning_status = STATUS.FIRSTPENDING;
	var marker = new BMap.Marker(drone_home);  // 创建标注
	this.map.addOverlay(marker);               // 将标注添加到地图中
	marker.setAnimation(BMAP_ANIMATION_BOUNCE); //跳动的动画
	//Switch dragging to disable because it's reading home position now
	/*marker.enableDragging();*/
	marker.disableDragging();

	var label = randomString(32);
	var bLabel = new BMap.Label(label);
	bLabel.setStyle({ display : "hidden"})
	marker.setTitle(label);
	this.markerList.push([label, marker, new wpOptions()]);

	me = this;
	this.addDragToMarker(marker);
	this.addMenuToMarker(marker);
	marker.addEventListener("mouseup",
	function (){
		console.log(me);
		if (me.planning_status == STATUS.FINISHED)
			return;
		me.planning_status = STATUS.FIRSTDONE;
		marker.setAnimation(null);
	})
}

Planner.prototype.addMarker = function(click_point) {
	//only add marker if in the right state (first marker been added )
	if (this.planning_status != STATUS.FIRSTDONE)
		return;
	var label;
	do
	{
		label = randomString(32);
	}
	while (this.checkLabelUnique(label) == false);

	// obtain a new label and add market
	var marker = new BMap.Marker(click_point);
	this.map.addOverlay(marker);
	marker.enableDragging();

	var bLabel = new BMap.Label(label);
	bLabel.setStyle({ display : "hidden"})
	marker.setTitle(label);
	this.markerList.push([label, marker, new wpOptions()]);

	// add line
	num_marker = this.markerList.length;
	var polyline = new BMap.Polyline([
		this.markerList[num_marker-2][1].point,
		this.markerList[num_marker-1][1].point,
	], {strokeColor:"blue", strokeWeight:3, strokeOpacity:0.5});   //创建折线
	polyline.disableEditing();
	this.map.addOverlay(polyline);
	this.lineList.push([label, polyline]);

	this.addDragToMarker(marker);
	this.addMenuToMarker(marker);

};

Planner.prototype.addMenuToMarker = function(marker) {
	me = this;

	var removeMarker = function(e,ee,marker){
		if (me.planning_status != STATUS.FIRSTDONE)
			return;
		me.map.removeOverlay(marker);
		var label = marker.getTitle();
		var idx = me.findLabelIndex(label);
		console.log("remove marker: "+label);

		if (idx == 0)
		{
			me.markerList.splice(idx,1);
			me.map.removeOverlay(me.lineList[idx][1]);
			me.lineList.splice(idx,1);
		}
		else
		{
			me.markerList.splice(idx,1);
			me.map.removeOverlay(me.lineList[idx-1][1]);
			me.lineList.splice(idx-1,1);
		}
		if (idx != me.markerList.length)
		{
			me.lineList[idx-1][1].setPath(
				[
					me.markerList[idx-1][1].point,
					me.markerList[idx][1].point,
				]
			);
		}

	};

	var setAltitude = function(e,ee,marker){
		var label = marker.getTitle();
		var idx = me.findLabelIndex(label);
		var altitude = prompt("Set Point Altitude", ""+me.markerList[idx][2].alti);
		me.markerList[idx][2].alti = parseFloat(altitude);

		renewMenu();
	};

	var setYaw = function(e,ee,marker) {
		var label = marker.getTitle();
		var idx = me.findLabelIndex(label);
		var yaw = prompt("Set Point Yaw", ""+me.markerList[idx][2].tgt_yaw);
		me.markerList[idx][2].tgt_yaw= yaw;

		renewMenu();
	};

	var setDampDis= function(e,ee,marker) {
		var label = marker.getTitle();
		var idx = me.findLabelIndex(label);
		var dampDis = prompt("Set Damping Dis", ""+me.markerList[idx][2].damping_dis);
		me.markerList[idx][2].damping_dis= dampDis;

		renewMenu();
	};

	var setGimbalPitch= function(e,ee,marker) {
		var label = marker.getTitle();
		var idx = me.findLabelIndex(label);
		var gimbalPitch = prompt("Set Gimbal Pitch", ""+me.markerList[idx][2].tgt_gimbal_pitch);
		me.markerList[idx][2].tgt_gimbal_pitch= gimbalPitch;

		renewMenu();
	};

	var setTurnMode= function(e,ee,marker) {
		var label = marker.getTitle();
		var idx = me.findLabelIndex(label);
		var turnMode = prompt("Set Turn Mode", ""+me.markerList[idx][2].turn_mode);
		me.markerList[idx][2].turn_mode = turnMode;

		renewMenu();
	};


	var setAction = function(){
		//TBD

	};

	//创建右键菜单

	var renewMenu= function() {
		var opts = {width:150};
		var markerMenu = new BMap.ContextMenu();
		markerMenu.addItem(new BMap.MenuItem('Delete waypoint', removeMarker.bind(marker), opts ));
		markerMenu.addSeparator();
		markerMenu.addItem(new BMap.MenuItem('Set altitude: ' + me.markerList[me.findLabelIndex(marker.getTitle())][2].alti, setAltitude.bind(marker),opts));
		markerMenu.addSeparator();
		/*
		markerMenu.addItem(new BMap.MenuItem('Set yaw angle: ' + me.markerList[me.findLabelIndex(marker.getTitle())][2].tgt_yaw, setYaw.bind(marker),opts));
		markerMenu.addSeparator();
		markerMenu.addItem(new BMap.MenuItem('Set damping dis: ' + me.markerList[me.findLabelIndex(marker.getTitle())][2].damping_dis, setDampDis.bind(marker),opts));
		markerMenu.addSeparator();
		markerMenu.addItem(new BMap.MenuItem('Set gimbal pitch: ' + me.markerList[me.findLabelIndex(marker.getTitle())][2].tgt_gimbal_pitch, setGimbalPitch.bind(marker),opts));
		markerMenu.addSeparator();
		markerMenu.addItem(new BMap.MenuItem('Set turning mode: ' + me.markerList[me.findLabelIndex(marker.getTitle())][2].turn_mode, setTurnMode.bind(marker),opts));
		markerMenu.addSeparator();
		markerMenu.addItem(new BMap.MenuItem('Set action', setAction.bind(marker),opts));
*/

		marker.addContextMenu(markerMenu);
	}

	renewMenu();
};

Planner.prototype.addDragToMarker = function(marker) {
	me = this;
	marker.addEventListener("dragging",
	function (){
		if (me.planning_status != STATUS.FIRSTDONE)
			return;
		var label = this.getTitle();
		var idx = me.findLabelIndex(label);
		console.log(label);
		if (idx != 0)
		{
			me.lineList[idx-1][1].setPath(
				[
					me.markerList[idx-1][1].point,
					me.markerList[idx][1].point,
				]
			);
		}
		if (idx != me.markerList.length-1)
		{
			me.lineList[idx][1].setPath(
				[
					me.markerList[idx][1].point,
					me.markerList[idx+1][1].point,
				]
			);
		}
	})
};

Planner.prototype.checkLabelUnique = function(label) {
	for(i = 0; i < this.markerList.length; i++)
	{
		if (label == this.markerList[i][0])
			return false;
	}
	return true;
};
Planner.prototype.findLabelIndex = function(label) {
	for(i = 0; i < this.markerList.length; i++)
	{
		if (label == this.markerList[i][0])
			return i;
	}
	return -1;
};

Planner.prototype.enableMapClick = function() {
	me = this;
	this.map.addEventListener("click", this.clickListener = function(e){
		if (me.planning_status == STATUS.FIRSTPENDING)
		{
			me.planning_status = STATUS.FIRSTDONE;
			me.markerList[0][1].setAnimation(null);
		}

		if (me.planning_status == STATUS.FIRSTDONE)
		{
			console.log(e.point.lng + "," + e.point.lat);
			me.addMarker(e.point);
		}
	});
};

Planner.prototype.isStandby = function() {
	return this.planning_status == STATUS.STANDBY;
}
Planner.prototype.isFinished = function() {
	return this.planning_status == STATUS.FINISHED;
}

Planner.prototype.confirmMission = function() {
	console.log(this);
	this.planning_status = STATUS.FINISHED;
	for(i = 0; i < this.markerList.length; i++)
	{
		this.markerList[i][1].disableDragging();
	}

	// write wp in monitor
	for(i = 0; i < this.markerList.length; i++)
	{
		var my_string = "<div>"+
			i + " " + this.markerList[i][1].point.lat + " " + this.markerList[i][1].point.lng + " " +
			this.markerList[i][2].alti
		+"</div>";
		$( my_string ).appendTo("#monitor");
	}
	me = this;
};

Planner.prototype.continuePlanning = function() {
	this.planning_status = STATUS.FIRSTDONE;
	for(i = 0; i < this.markerList.length; i++)
	{
		this.markerList[i][1].enableDragging();
	}
};
