var KEYBOARDTELEOP = KEYBOARDTELEOP || {
	REVISION : '3'
};

KEYBOARDTELEOP.Teleop = function(options) {
	var that = this;
	options = options || {};
	var ros = options.ros;
	var topic = options.topic || '/cmd_vel'; // should be /board_ctrl?

	var cmd_list = new Array(
		'NULL_COMMAND', 'GO_HOME','EXIT_GO_HOME','EXIT_GH_LANDING','AUTO_TAKEOFF','EXIT_TAKEOFF',
		'AUTO_LANDING', 'EXIT_LANDING', 'START_MOTOR', 'STOP_MOTOR', 'START_CALICOM', 'STOP_CALICOM',
		'ENABLE_IOC_MODE', 'DISABLE_IOC_MODE', 'SET_HOTPOINT', 'SET_COURSE', 'SET_A_HOMEPOINT', 'SET_C_HOMEPOINT',
		'ENTER_PACK_MODE', 'EXIT_PACK_MODE', 'GEAR_HIDE', 'GEAR_SHOW');

	var nav_list = new Array("open","close");

	// linear x and y movement and angular z movement
	var x = 0;
	var y = 0;
	var z = 0;
	var speed_level = 1;
	var ctrl_mode 	= 1;
	var base_speed = 2;
	var base_vel_speed = 0.1;
	var height_speed = 0;

	var my_request = 3;

	// publisher to change status
	var flight_status_publisher = new ROSLIB.Topic({
		ros : ros,
		name : '/sdk_request_cmd',
		messageType : 'std_msgs/Float32'
	});

	// publisher to change nav open close
	var nav_open_close_request_publisher = new ROSLIB.Topic({
		ros : ros,
		name : '/nav_open_close_request',
		messageType : 'std_msgs/Float32'
	});

	// publisher to send control command
	var cmdVel = new ROSLIB.Topic({
		ros : ros,
		name : topic, // '/cmd_vel'
		messageType : 'geometry_msgs/Quaternion'
	});

	// publisher to send ctrl mode
	var ctrl_mode_publisher = new ROSLIB.Topic({
		ros : ros,
		name : '/sdk_request_ctrl_mode',
		messageType : 'std_msgs/Float32'
	});

	// publisher to change task
	var task_start_stop_request_publisher = new ROSLIB.Topic({
		ros : ros,
		name : '/sdk_request_simple_task',
		messageType : 'std_msgs/Float32'
	});

	// publisher to change nav open close
	var activation_request_publisher = new ROSLIB.Topic({
		ros : ros,
		name : '/sdk_request_activation',
		messageType : 'std_msgs/Float32'
	});

	// sets up a key listener on the page used for keyboard tele-operation
	var handleKey = function(keyCode, keyDown) {
		// used to check for changes in speed
		var oldX = x;
		var oldY = y;
		var oldZ = z;

		var speed = 0;
		var iskeydown = 0;
		// throttle the speed by the slider and throttle constant
		if (keyDown === true) {
			iskeydown = 1;
		}
		else 
			iskeydown = 0;
		// check which key was pressed
		console.log(keyCode)
		switch (keyCode) {
		case 65:    //key A
			// roll positive or x
			if(ctrl_mode == 1)
				y = -base_speed*speed_level*iskeydown;
			else if(ctrl_mode == 2)
				x = -base_vel_speed*speed_level*iskeydown;
			break;
		case 87:   // key W
			// pitch positive or y
			if(ctrl_mode == 1)
				x = -base_speed*speed_level*iskeydown;
			// +x
			else if(ctrl_mode == 2)
				y = base_vel_speed*speed_level*iskeydown;
			break;
		case 68:   //key D
			// roll negative or -x
			if(ctrl_mode == 1)
				y = base_speed*speed_level*iskeydown;
			else if(ctrl_mode == 2)
				x = base_vel_speed*speed_level*iskeydown;
			break;
		case 83:   // key S
			// pitch negative or -y
			if(ctrl_mode == 1)
				x = base_speed*speed_level*iskeydown;
			else if(ctrl_mode == 2)
				y = -base_vel_speed*speed_level*iskeydown;
			break;
		case 69:   // key E
			// yaw rate positive
			z = base_speed*speed_level*iskeydown*3;
			break;
		case 81:   // key Q
			// yaw rate negative
			z = -base_speed*speed_level*iskeydown*3;
			break;
			
		case 90:  // key Z
			height_speed = 0.30*iskeydown;   // 0.3 * 100
			break;
		case 67:  // key C
			height_speed = -0.30*iskeydown;  // 0.3 * 100
			break;

			// change speed level
		case 49: // num 1
			speed_level = 1;
			break;
		case 50: // num 2
			speed_level = 2;
			break;
		case 51: // num 3
			speed_level = 3;
			break;
		case 52: // num 4
			speed_level = 4;
			break;
		case 53: // num 5
			speed_level = 5;
			break;
		case 54: // num 6
			speed_level = 6;
			break;
		case 56: // num 8
			ctrl_mode = 1;
			var request = new ROSLIB.Message({data : 1,});
   			ctrl_mode_publisher.publish(request);
			break;
		case 57: // num 9
			ctrl_mode = 2;
			var request = new ROSLIB.Message({data : 2,});
    			ctrl_mode_publisher.publish(request);
			break;
		}
		var speedIndicator = document.getElementById('speed-level');
		var modeIndicator = document.getElementById('ctrl-mode');
		speedIndicator.textContent = String(speed_level);
		modeIndicator.textContent = String(ctrl_mode);

		// publish the command
		var qua = new ROSLIB.Message({
			x : x,
			y : y,
			z : z,
			w : height_speed
		});
		cmdVel.publish(qua);

		// check for changes
		if (oldX !== x || oldY !== y || oldZ !== z) {
			that.emit('change', qua);
		}
	};

	// handle the key
	var body = document.getElementsByTagName('body')[0];
	body.addEventListener('keydown', function(e) {
		handleKey(e.keyCode, true);
	}, false);
	body.addEventListener('keyup', function(e) {
		handleKey(e.keyCode, false);
	}, false);
	
	// handle takeoff and landing
	var navbutton = document.getElementById("open-close-nav-button");
        var navreqview= document.getElementById("nav-open-close-request");
	var takeoffbutton = document.getElementById("takeoff-button");
	var landingbutton = document.getElementById("landing-button");
	var gohomebutton = document.getElementById("gohome-button");
	var cmd_request = document.getElementById('cmd-request');


	var taskbutton = document.getElementById("task-button");
	var taskIndicator = document.getElementById('task-onoff');

	var activationbutton = document.getElementById("activation-button");


	var display_request = function (cmd_num) {
		while( cmd_request.firstChild ) {
    		cmd_request.removeChild( cmd_request.firstChild );
		}
		cmd_request.appendChild( document.createTextNode(cmd_list[cmd_num]) );
	};
	display_request(0);

	
	var nav_open_close_display_request = function (req_num) {
		while( navreqview.firstChild ) {
    		navreqview.removeChild( navreqview.firstChild );
		}
		navreqview.appendChild( document.createTextNode(nav_list[req_num]) );
	};
	nav_open_close_display_request(0);

	var nav_open_close_flag = 0;
	navbutton.addEventListener('mousedown', function(e) {
		// publish status change request
		if(nav_open_close_flag == 0)
		{
			nav_open_close_flag = 1;
			nav_open_close_display_request(1);
			var request = new ROSLIB.Message({
  			data : 1,});
			
		}
		else
		{
			nav_open_close_flag = 0;
			nav_open_close_display_request(0);
			var request = new ROSLIB.Message({
  			data : 0,});
		}
		nav_open_close_request_publisher.publish(request);
		
	},false);	


	takeoffbutton.addEventListener('mousedown', function(e) {
		// publish status change request
		var request = new ROSLIB.Message({
  			data : 4,
		});
		
		//for (i=0;i<10;i++)
		//{
    		flight_status_publisher.publish(request);
		//}
		console.log("4");
		display_request(4);
		
	},false);
	takeoffbutton.addEventListener('mouseup', function(e) {
		// publish status change request
		//var request = new ROSLIB.Message({
  		//	data : 3,
		//});
		//flight_status_publisher.publish(request);
		//console.log("3");
		//display_request(3);
		
	},false);
	landingbutton.addEventListener('mousedown', function(e) {
		// publish status change request
		var request = new ROSLIB.Message({
  			data : 6,
		});
		flight_status_publisher.publish(request);
		console.log("6");
		display_request(6);
	},false);
	landingbutton.addEventListener('mouseup', function(e) {
		// publish status change request
		//var request = new ROSLIB.Message({
  		//	data : 3,
		//});
		//flight_status_publisher.publish(request);
		//console.log("3");
		//display_request(3);
	},false);

	gohomebutton.addEventListener('mousedown', function(e) {
		// publish status change request
		var request = new ROSLIB.Message({
  			data : 1,
		});
		
		//for (i=0;i<10;i++)
		//{
    		flight_status_publisher.publish(request);
		//}
		console.log("1");
		display_request(1);
		
	},false);


	var task_flag = 0;
	taskIndicator.textContent = String("stoped");
	taskbutton.addEventListener('mousedown', function(e) {
		// publish status change request
		if(task_flag == 0)
		{
			task_flag = 1;
			taskIndicator.textContent = String("started");
			var request = new ROSLIB.Message({
  			data : 4,});
			
		}
		else
		{
			task_flag = 0;
			taskIndicator.textContent = String("stoped");
			var request = new ROSLIB.Message({
  			data : -1,});
		}
		task_start_stop_request_publisher.publish(request);
		
	},false);	

	activationbutton.addEventListener('mousedown', function(e) {

		var request = new ROSLIB.Message({data : 1,});
		activation_request_publisher.publish(request);
		
	},false);

};
KEYBOARDTELEOP.Teleop.prototype.__proto__ = EventEmitter2.prototype;
