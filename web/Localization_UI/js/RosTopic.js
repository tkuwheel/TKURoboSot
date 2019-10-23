//=====================================================================================
// hold ball
var HoldBall1 = new ROSLIB.Topic({
    ros: ros,
    name: 'motion/hold_ball',
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
//=====================================================================================
//ball
var ball = new ROSLIB.Topic({
    ros: ros,
    name: 'vision/object',
    messageType: '/vision/Object'
});
ball.subscribe(function(msg) {
    ball_distance = msg.ball_dis;
    ball_angle = msg.ball_ang;

    document.getElementById('ball_ang').innerText = Math.round(ball_angle)+" °";
    document.getElementById('ball_dis').innerText = Math.round(ball_distance)+" cm";
    //imu_3d_w=-msg.yaw+90/180*Math.PI;
    //console.log(msg.yaw);
});
//=======================================================
var imu_3d = new ROSLIB.Topic({
    ros: ros,
    name: 'imu_3d',
    messageType: 'imu_3d/inertia'
});
imu_3d.subscribe(function(msg) {
    imu_3d_w=-msg.yaw+(90/180*Math.PI);
    //console.log(msg.yaw);
});
//=======================================================
var path = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/PathOrder',
    messageType: 'std_msgs/Float32MultiArray'
});
path.subscribe(function(msg) {
    let count=0;
    if(get_path==true){
        for(let i=0; i<msg.data.length; i+=2){
            
            if(msg.data[i]!=0||msg.data[i+1]!=0){
                //console.log(msg.data[i],msg.data[i+1]);
                Path[count]=Math.floor(msg.data[i]*1000)/10;
                count++;
                Path[count]=Math.floor(msg.data[i+1]*1000)/10;
                count++;
            }
        }
        ClearAllRectangle();
        
        let canvas = document.getElementById('path_map');
        let ground_reverse = document.getElementById("GroundButton").checked;
        let center_x = canvas.width/2;
        let center_y = canvas.height/2;
        let ctx=canvas.getContext("2d");
        ctx.clearRect(0,0,canvas.width,canvas.height); 
        for(let i=0; i<Path.length; i+=2){
            console.log(Path[i],Path[i+1]);

            ctx.beginPath();
            ctx.font = "50px Times New Roman";
            ctx.fillStyle="white";
            if(ground_reverse==false){
              ctx.fillText(i/2+1,center_x+Path[i]*1.3-10,center_y-Path[i+1]*1.3+15);
            }else{
              ctx.fillText(i/2+1,center_x-Path[i]*1.3-10,center_y+Path[i+1]*1.3+15);
            }
            ctx.closePath();
        }
    }
    get_path=false;
});
//=======================================================

var coord = new ROSLIB.Topic({
    ros: ros,
    name: 'akf_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
});
coord.subscribe(function(msg) {
    let qx=msg.pose.pose.orientation.x;
    let qy=msg.pose.pose.orientation.y;
    let qz=msg.pose.pose.orientation.z;
    let qw=msg.pose.pose.orientation.w;
    let pi=Math.PI;
    //console.log(qx,qy,qz,qw);
    let x_,y_;
    x=Math.round(msg.pose.pose.position.x*100);
    y=Math.round(msg.pose.pose.position.y*100);
    w=Math.round(Math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)/pi*180);
    //console.log(x,y,w);
    
    let angle = (w)/180*pi;
    x_= x+20 * Math.cos(angle);
    y_= y+20 * Math.sin(angle);
    x_=Math.round(x_);
    y_=Math.round(y_);

    let x_imu,y_imu,angle_imu;
    angle_imu = imu_3d_w;
    x_imu= x+20 * Math.cos(angle_imu);
    y_imu= y+20 * Math.sin(angle_imu);
    x_imu=Math.round(x_imu);
    y_imu=Math.round(y_imu);

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
    let whitelinemap =  document.getElementById("WhitelineButton").checked;
    if(!map_checked&&!whitelinemap){
        ctx.beginPath();
        if(sigma<20){
            ctx.lineWidth = 5;
            ctx.strokeStyle = '#FFFF00';
        }else{
            ctx.lineWidth = 10;
            ctx.strokeStyle = '#FF0000';
        }
        if(ground_reverse==false){
            ctx.beginPath();
            ctx.arc(center_x+(x*1.3), center_y-(y*1.3), 23, 0, 2*Math.PI);
            ctx.moveTo(center_x+(x*1.3), center_y-(y*1.3));
            ctx.lineTo(center_x+(x_*1.3), center_y-(y_*1.3));
            ctx.stroke();
            ctx.closePath();
            //==========imu============
            ctx.beginPath();
            ctx.strokeStyle = '#FF0000';
            ctx.lineWidth = 3;
            ctx.moveTo(center_x+(x*1.3), center_y-(y*1.3));
            ctx.lineTo(center_x+(x_imu*1.3), center_y-(y_imu*1.3));
            ctx.stroke();
            ctx.closePath();
            
            //===========ball===========
            if(ball_angle!=999){
              ctx.beginPath();
              let x_ball,y_ball,angle_ball;
              angle_ball = (ball_angle+w)/180*pi;
              x_ball= x+ball_distance * Math.cos(angle_ball);
              y_ball= y+ball_distance * Math.sin(angle_ball);
              x_ball=Math.round(x_ball);
              y_ball=Math.round(y_ball);
              //ctx.lineWidth = 1;
              //ctx.strokeStyle = '#FFFF00';
              //ctx.moveTo(center_x+(x*1.3), center_y-(y*1.3));
              //ctx.lineTo(center_x+(x_ball*1.3), center_y-(y_ball*1.3));
              ctx.arc(center_x + (x_ball * 1.3), center_y - (y_ball * 1.3),15,0,360,false);
              ctx.fillStyle="red";//填充颜色,默认是黑色
              ctx.fill();//画实心圆
              ctx.closePath();
              ctx.beginPath();
              ctx.strokeStyle = '#000000';
              ctx.arc(center_x+(x_ball*1.3), center_y-(y_ball *1.3), 15, 0, 2*Math.PI);
              ctx.stroke();
              ctx.closePath();
            }
        }else{
            ctx.beginPath();
            ctx.arc(center_x-(x*1.3), center_y+(y*1.3), 23, 0, 2*Math.PI);
            ctx.moveTo(center_x-(x*1.3), center_y+(y*1.3));
            ctx.lineTo(center_x-(x_*1.3), center_y+(y_*1.3));
            ctx.stroke();
            ctx.closePath();
            //===========imu============
            ctx.beginPath();
            ctx.strokeStyle = '#FF0000';
            ctx.lineWidth = 3;
            ctx.moveTo(center_x-(x*1.3), center_y+(y*1.3));
            ctx.lineTo(center_x-(x_imu*1.3), center_y+(y_imu*1.3));
            ctx.stroke();
            ctx.closePath();
            //===========ball===========
            if(ball_angle!=999){
              ctx.beginPath();
              let x_ball,y_ball,angle_ball;
              angle_ball = (ball_angle+w)/180*pi;
              x_ball= x+ball_distance * Math.cos(angle_ball);
              y_ball= y+ball_distance * Math.sin(angle_ball);
              x_ball=Math.round(x_ball);
              y_ball=Math.round(y_ball);
              //ctx.lineWidth = 1;
              //ctx.strokeStyle = '#FFFF00';
              //ctx.moveTo(center_x-(x*1.3), center_y+(y*1.3));
              //ctx.lineTo(center_x-(x_ball*1.3), center_y+(y_ball*1.3));
              ctx.arc(center_x - (x_ball * 1.3), center_y + (y_ball * 1.3),15,0,360,false);
              ctx.fillStyle="red";//填充颜色,默认是黑色
              ctx.fill();//画实心圆
              ctx.closePath();
              ctx.beginPath();
              ctx.strokeStyle = '#000000';
              ctx.arc(center_x-(x_ball*1.3), center_y+(y_ball*1.3), 15, 0, 2*Math.PI);
              ctx.stroke();
              ctx.closePath();
            }
        }
        //ctx.stroke();
    }
});
var std = new ROSLIB.Topic({
    ros: ros,
    name: 'mcl/std',
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
var save_parameter = new ROSLIB.Topic({
    ros: ros,
    name: 'mcl/save',
    messageType: 'std_msgs/Empty'
});
function saveyamel(){
    var msg = new ROSLIB.Message({
    });
    //console.log(msg.w);
    save_parameter.publish(msg); 
}
/*========================================================*/
//MotionRemote
var Remote = new ROSLIB.Topic({
    ros: ros,
    name: 'motion/remote',
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
var chase_pub = new ROSLIB.Topic({
    ros: ros,
    name: '/localization/chase_enable',
    messageType: 'std_msgs/Int32'
});
function chase_switch() {
    let enable = document.getElementById('ChaseButton').checked;
    let chase_enable = 0;
    if(enable){
        chase_enable = 1;
        console.log("追球開啟");
    }else{
        console.log("追球關閉");
    }
    var msg = new ROSLIB.Message({
        data:chase_enable
    });
    chase_pub.publish(msg);
}
/*========================================================*/
var imu_pub = new ROSLIB.Topic({
    ros: ros,
    name: 'imu_3d/angle_correction',
    messageType: 'std_msgs/Float32'
});
function ImuReset() {
    var msg = new ROSLIB.Message({
        data:w-90
    });
    console.log(w-90);
    imu_pub.publish(msg);
}
/*========================================================*/
var resetParticles = new ROSLIB.Topic({
    ros: ros,
    name: 'mcl/resetParticles',
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
    name: 'FIRA/Location',
    messageType: 'std_msgs/Float32MultiArray'
});
//Optimization
var OptimizationBox = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/Optimization',
    messageType: 'std_msgs/Int32MultiArray'
});
//GameState
var GameState = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var SaveParam = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});
//cmd_vel
var cmdVel1 = new ROSLIB.Topic({
    ros: ros,
    name: 'motion/cmd_vel',
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
    if(state==9){
        get_path=true;
    }
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
function stop_robot(){
    var twist = new ROSLIB.Message({
        linear: {
            x: 0,
            y: 0,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: 0
        }
    });
    console.log(stop);
    cmdVel1.publish(twist);
}
