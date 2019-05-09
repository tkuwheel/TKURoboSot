//=====================================================================================
// hold ball
var HoldBall1 = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/hold_ball',
    messageType: 'std_msgs/Bool'
});

function HoldBallSwitch(state,robot) {
    var check;
    if (state) {
        console.log(robot,"hold ball :",state);
        check = new ROSLIB.Message({
            data: true
        });
    } else {
        console.log(robot,"hold ball :",state);
        check = new ROSLIB.Message({
            data: false
        });
    }
    HoldBall1.publish(check);
}
//=======================================================
var coord = new ROSLIB.Topic({
    ros: ros,
    name: '/akf_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
});
coord.subscribe(function(msg) {
    let qx=msg.pose.pose.orientation.x;
    let qy=msg.pose.pose.orientation.y;
    let qz=msg.pose.pose.orientation.z;
    let qw=msg.pose.pose.orientation.w;
    let pi=3.1415;
    //console.log(qx,qy,qz,qw);
    let x_,y_;
    x_=Math.round(msg.pose.pose.position.x*100);
    y_=Math.round(msg.pose.pose.position.y*100);
    w=Math.round(Math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)/pi*180);
    //console.log(x,y,w);
    
    let angle = (w-180)/180*pi;
    x= x_+20 * Math.cos(angle);
    y= y_+20 * Math.sin(angle);
    x=Math.round(x);
    y=Math.round(y);

    document.getElementById('coordinate_x').innerText = x;
    document.getElementById('coordinate_y').innerText = -y;
    document.getElementById('coordinate_angle').innerText = w;

    let canvas = document.getElementById('robot_map');
    let center_x = canvas.width/2;
    let center_y = canvas.height/2;

    let ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height);  
    let map_checked=document.getElementById('LocalizationButton').checked;
    let ground_reverse = document.getElementById("GroundButton").checked;
    if(!map_checked){
        ctx.beginPath();
        if(sigma<60){
            ctx.lineWidth = 5;
            ctx.strokeStyle = '#FFFF00';
        }else{
            ctx.lineWidth = 5;
            ctx.strokeStyle = '#FF0000';
        }
        if(ground_reverse==false){
            ctx.arc(center_x+(x*1.3), center_y-(y*1.3), 23, 0, 2*Math.PI);
            ctx.moveTo(center_x+(x*1.3), center_y-(y*1.3));
            ctx.lineTo(center_x+(x_*1.3), center_y-(y_*1.3));
        }else{
            ctx.arc(center_x-(x*1.3), center_y+(y*1.3), 23, 0, 2*Math.PI);
            ctx.moveTo(center_x-(x*1.3), center_y+(y*1.3));
            ctx.lineTo(center_x-(x_*1.3), center_y+(y_*1.3));
        }
        ctx.stroke();
    }
});
var std = new ROSLIB.Topic({
    ros: ros,
    name: '/mcl/std',
    messageType: 'std_msgs/Float32'
});
std.subscribe(function(msg) {
    sigma=msg.data;
    document.getElementById('coordinate_sigma').innerText = Math.round(sigma);
    if(sigma>60){
        document.getElementById('coordinate_sigma').style.color="#FF0000";
    }else{
        document.getElementById('coordinate_sigma').style.color="#000000";
    }
});
/*========================================================*/
//MotionRemote
var Remote = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/remote',
    messageType: 'std_msgs/Bool'
});
//Remote_state
var RemoteState = 0;
function RemoteSwitch(state) {
    var check;
    if (state) {
        console.log("Remote on");
        RemoteState = true;
        check = new ROSLIB.Message({
            data: RemoteState
        });
    } else {
        console.log("Remote off");
        RemoteState = false;
        check = new ROSLIB.Message({
            data: RemoteState
        });
    }
    Remote.publish(check);
}
/*========================================================*/

/*========================================================*/
var resetParticles = new ROSLIB.Topic({
    ros: ros,
    name: '/mcl/resetParticles',
    messageType: 'self_localization/resetParticles'
});
function CoordReset() {
    let canvas = document.getElementById('reset_map');
    let ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height);

    var msg = new ROSLIB.Message({
        init: reset_bool,
        x: reset_x,
        y: reset_y,
        w: reset_w
    });
    //console.log(msg.w);
    resetParticles.publish(msg);
    reset_bool=false;
}
function CoordReverse() {
    var msg = new ROSLIB.Message({
        init: true,
        x: -x,
        y: y,
        w: w-180
    });
    //console.log(msg.w);
    resetParticles.publish(msg);
}
/*========================================================*/
//Region
var RegionBox = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/Location',
    messageType: 'std_msgs/Float32MultiArray'
});
//Optimization
var OptimizationBox = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/Optimization',
    messageType: 'std_msgs/Int32MultiArray'
});
//GameState
var GameState = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var SaveParam = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});
//cmd_vel
var cmdVel1 = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
//TopicFunction
function TopicRegion(Num) {
    var box = new ROSLIB.Message({
        data: [Num[0], Num[1], Num[2], Num[3], Num[4], Num[5], Num[6], Num[7], Num[8], Num[9]]
    });
    RegionBox.publish(box);
}
function TopicOptimization(Num) {
    var box = new ROSLIB.Message({
        data: [Num[0], Num[1], Num[2], Num[3]]
    });
    OptimizationBox.publish(box);
}
function topicROSGameState(state) {
    console.log(state);
    var gameState = new ROSLIB.Message({
        data: state
    });
    GameState.publish(gameState);
}
function OpenSimulator(checked) {
    var temp;
    if (checked == true) {
        // temp = new ROSLIB.Message({
        //     data: 1
        // });
        // IsSimulator.publish(temp);
        temp = 1;
        IsSimulator.set(temp);
        console.log(temp);
    } else {
        // temp = new ROSLIB.Message({
        //     data: 0
        // });
        // IsSimulator.publish(temp);
        temp = 0;
        IsSimulator.set(temp);
        console.log(temp);
    }
}
function topicSaveParam(value){
    console.log(value);
    var Param = new ROSLIB.Message({
        data:value
    })
    SaveParam.publish(Param);
}
function PublishTopicCmdVel(vec3) {
    var twist = new ROSLIB.Message({
        linear: {
            x: vec3.x,
            y: vec3.y,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: vec3.z
        }
    });
    if (RemoteState) {
      console.log(twist);
      cmdVel1.publish(twist);
    }
}
