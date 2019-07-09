function topic_creater(topic_name , type){
	var topic_base = new ROSLIB.Topic({
		ros : ros,
		name : topic_name,
		messageType : type
	});
	console.log(topic_name+" Topic Creat!");
	return topic_base;
}

function topic_cmdVel_pub(_x,_y,_z){
	var cmdVel = new ROSLIB.Topic({
		ros : ros,
		name : '/cmd_vel',
		messageType : 'geometry_msgs/Twist'
	});
	console.log('/cmd_vel creat');
	var twist = new ROSLIB.Message({
		linear : {
			x : _x,
			y : _y,
			z : 0
		},
		angular : {
			x : 0,
			y : 0,
			z : _z
		}
	});
	cmdVel.publish(twist);
}

function topic_cmdVel_pub_1(){
	var cmdVel = new ROSLIB.Topic({
		ros : ros,
		name : '/test',
		messageType : 'std_msgs/Char'
	});
	console.log('/test creat');
	var twist = new ROSLIB.Message({
		data : 97
	});
	cmdVel.publish(twist);
}
