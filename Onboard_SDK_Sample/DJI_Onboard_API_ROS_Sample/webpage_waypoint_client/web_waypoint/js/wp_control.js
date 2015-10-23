
var dji_home= new BMap.Point(113.958004, 22.542494);

$( document ).ready(function() {
	// read drone info from ws
	// use a fixed point for testing purpose
	/*var drone_home = new BMap.Point(113.958004, 22.542494);*/

	//TODO: re-organize this things
	//my_socket = new WebSocket("ws://localhost:19870/controller/navigation/");
	my_socket = new WebSocket("ws://localhost:19871");
	my_Communicator= new Communicator(my_socket);

	// create map
	map = new BMap.Map("allmap");
	map.centerAndZoom(dji_home, 15);
	map.enableScrollWheelZoom();
	map.disableDoubleClickZoom();
	map.enableKeyboard();
	// create planner
	my_planner = new Planner(map);
	my_planner.reset();
/*
	init_socket = new WebSocket("ws://localhost:19870/general");

	init_socket.onmessage = function(event) {
		console.log(event.data);
		var msg = JSON.parse(event.data);
		var file = msg.FILE;
		var device_type = msg.DEVICE_TYPE;
		var event = msg.EVENT;
		var product_type = msg.PRODUCT_TYPE;
		var version = msg.VERSION;

		my_socket = new WebSocket("ws://localhost:19870/controller/navigation/"+file);
		my_Communicator= new Communicator(my_socket);
	};
*/

	//bind events to control panel wadgets
	$( "#start-plan" ).bind( "click", function() {

		//my_Communicator.getGlobalPosition();
		console.log('Home position: ' + home_lon + ', ' + home_lat);
		if(home_lon == 0 || home_lat == 0)
			alert("Home Location Not Recorded!");
			return;
			
		var drone_home = new BMap.Point(home_lon, home_lat);
		map.setCenter(drone_home);

		if (my_planner.isStandby())
		{
			my_planner.addFirstMarker(drone_home);
			my_planner.enableMapClick();
		}
		else if (my_planner.isFinished())
		{
			my_planner.continuePlanning();
		}
	});

	$("#confirm-mission").bind("click", function() {
		my_planner.confirmMission();
	});

        $("#reset-mission").bind("click", function() {
		my_planner.reset();
                $("#monitor").empty();
	});

	$("#upload-mission").bind("click", function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.uploadWayline();
	});

	$("#open-navmode").bind("click", function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.setNavigationMode();
	});

	$("#start-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.startWayline();

	});
/*
    $("#download-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.downloadWayline();
	});

    $("#download-waypoint").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
            my_Communicator.downloadWaypoint(1);
	});

	$("#set-idle").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.setWaylineIdelValue(4);
	});

    $("#get-idle").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.getWaylineIdelValue();
	});

    $("#pause-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.pauseWayline();

	});

    $("#resume-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.continueWayline();

	});
*/
	$("#cancel-mission").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.cancelWayline();

	});

	$("#close-navmode").bind("click",function() {
		if (typeof(my_Communicator) == 'undefined')
			alert("Drone not Connected!");
		else
			my_Communicator.stopNavigationMode();

	});
	$("#login").bind("click",function() {

	});

	$("#update").bind("click",function() {

	});

	$("#setSpeed").bind("click",function() {

	});
});

