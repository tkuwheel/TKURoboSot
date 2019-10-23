
/*========================================================*/
//sigma
var s1 = new ROSLIB.Topic({
    ros: ros,
    name: 'mcl/std',
    messageType: 'std_msgs/Float32'
});
var s2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'mcl/std',
    messageType: 'std_msgs/Float32'
});
var s3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'mcl/std',
    messageType: 'std_msgs/Float32'
});
//sigma
s1.subscribe(function (msg) {
    sigma1 = msg.data;
    //console.log(sigma);
    document.getElementById('coordinate_sigma1').innerText = Math.round(sigma1);
    if (sigma1 > 20) {
        document.getElementById('coordinate_sigma1').style.color = "#FF0000";
    } else {
        document.getElementById('coordinate_sigma1').style.color = "#fff";
    }
});
s2.subscribe(function (msg) {
    sigma2 = msg.data;
    document.getElementById('coordinate_sigma2').innerText = Math.round(sigma2);
    if (sigma2 > 20) {
        document.getElementById('coordinate_sigma2').style.color = "#FF0000";
    } else {
        document.getElementById('coordinate_sigma2').style.color = "#fff";
    }
});
s3.subscribe(function (msg) {
    sigma3 = msg.data;
    document.getElementById('coordinate_sigma3').innerText = Math.round(sigma3);
    if (sigma3 > 20) {
        document.getElementById('coordinate_sigma3').style.color = "#FF0000";
    } else {
        document.getElementById('coordinate_sigma3').style.color = "#fff";
    }
});
/*========================================================*/
//imu
var s1 = new ROSLIB.Topic({
    ros: ros,
    name: 'imu_3d',
    messageType: 'imu_3d/inertia'
});
var s2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'imu_3d',
    messageType: 'imu_3d/inertia'
});
var s3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'imu_3d',
    messageType: 'imu_3d/inertia'
});
//sigma
s1.subscribe(function (msg) {
    imu1=-msg.yaw+90/180*Math.PI;
});
s2.subscribe(function (msg) {
    imu2=-msg.yaw+90/180*Math.PI;
});
s3.subscribe(function (msg) {
    imu3=-msg.yaw+90/180*Math.PI;
});
/*========================================================*/

//Map
var V1 = new ROSLIB.Topic({
    ros: ros,
    name: 'akf_pose',
    messageType: '/geometry_msgs/PoseWithCovarianceStamped'
    
});
var V2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'akf_pose',
    messageType: '/geometry_msgs/PoseWithCovarianceStamped'
    
});
var V3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'akf_pose',
    messageType: '/geometry_msgs/PoseWithCovarianceStamped'
    
});

//Map
V1.subscribe(function(msg) {
    //console.log(msg.pose.pose.position.x)
   
    let qx=msg.pose.pose.orientation.x;
    let qy=msg.pose.pose.orientation.y;
    let qz=msg.pose.pose.orientation.z;
    let qw=msg.pose.pose.orientation.w;
    let pi=3.1415;
    //console.log(qx,qy,qz,qw);
    //let x_,y_;
    x1=Math.round(msg.pose.pose.position.x*100);
    y1=Math.round(msg.pose.pose.position.y*100);
    w1=Math.round(Math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)/pi*180);
    //console.log(x_,y_,w);
    
    let angle = (w1)/180*pi;
    x1_= x1+20 * Math.cos(angle);
    y1_= y1+20 * Math.sin(angle);
    x1_=Math.round(x1_);
    y1_=Math.round(y1_);

    document.getElementById('coordinate_x1').innerText = x1;
    document.getElementById('coordinate_y1').innerText = y1;
    document.getElementById('coordinate_angle1').innerText = w1;
 
    draw_robot_map();
});
V2.subscribe(function(msg) {
    
    //console.log(msg.pose.pose.position.x)
   
    let qx=msg.pose.pose.orientation.x;
    let qy=msg.pose.pose.orientation.y;
    let qz=msg.pose.pose.orientation.z;
    let qw=msg.pose.pose.orientation.w;
    let pi=3.1415;
    //console.log(qx,qy,qz,qw);
    //let x_,y_;
    x2=Math.round(msg.pose.pose.position.x*100);
    y2=Math.round(msg.pose.pose.position.y*100);
    w2=Math.round(Math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)/pi*180);
    //console.log(x_,y_,w);
    
    let angle = (w2)/180*pi;
    x2_= x2+20 * Math.cos(angle);
    y2_= y2+20 * Math.sin(angle);
    x2_=Math.round(x2_);
    y2_=Math.round(y2_);

    document.getElementById('coordinate_x2').innerText = x2;
    document.getElementById('coordinate_y2').innerText = y2;
    document.getElementById('coordinate_angle2').innerText = w2;
    
    draw_robot_map();
});
V3.subscribe(function(msg) {
   
    //console.log(msg.pose.pose.position.x)
   
    let qx=msg.pose.pose.orientation.x;
    let qy=msg.pose.pose.orientation.y;
    let qz=msg.pose.pose.orientation.z;
    let qw=msg.pose.pose.orientation.w;
    let pi=3.1415;
    //console.log(qx,qy,qz,qw);
    //let x_,y_;
    x3=Math.round(msg.pose.pose.position.x*100);
    y3=Math.round(msg.pose.pose.position.y*100);
    w3=Math.round(Math.atan2(2 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)/pi*180);
    //console.log(x_,y_,w);
    
    let angle = (w3)/180*pi;
    x3_= x3+20 * Math.cos(angle);
    y3_= y3+20 * Math.sin(angle);
    x3_=Math.round(x3_);
    y3_=Math.round(y3_);

    document.getElementById('coordinate_x3').innerText = x3;
    document.getElementById('coordinate_y3').innerText = y3;
    document.getElementById('coordinate_angle3').innerText = w3;

    draw_robot_map();
});

function draw_robot_map(){
    let canvas = document.getElementById("robot_map");
    let ctx = canvas.getContext("2d");
    
    let center_x = canvas.width / 2;
    let center_y = canvas.height / 2;
    
    let map_checked = document.getElementById("CameraSwitch").checked;
    let ground_reverse = document.getElementById("GroundButton").checked;
    let MCLmap_checked = document.getElementById("MCLmap").checked;

    if(!map_checked){
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        var sigma_error = 20;
        
        ctx.beginPath();

        //draw_robot(1,sigma1,x1,y1,w1,catchball1,imu1);
        draw_robot(2,sigma2,x2,y2,w2,catchball2,imu2);
        draw_robot(3,sigma3,x3,y3,w3,catchball3,imu3);
        ball_condition();
        draw_obstacle(x1,y1,w1,obstacle_info1);
        draw_obstacle(x2,y2,w2,obstacle_info2);
        draw_obstacle(x3,y3,w3,obstacle_info3);
        draw_move_line(x1,y1,w1,move_angle1,move_speed1,move_w1);
        draw_move_line(x2,y2,w2,move_angle2,move_speed2,move_w2);
        draw_move_line(x3,y3,w3,move_angle3,move_speed3,move_w3);
        
    }
}
function draw_obstacle(x,y,w,obstacle_info){
    
    let canvas = document.getElementById("robot_map");
    let ctx = canvas.getContext("2d");
    let center_x = canvas.width / 2;
    let center_y = canvas.height / 2;
    let angle = (w)/180*Math.PI;
    ctx.strokeStyle = "#000";
    ctx.lineWidth = 5;
    for(let i=0; i<obstacle_info.length; i+=4){
        ctx.beginPath();
        if(w!=0){
            let o_dis = obstacle_info[i+0];
            let o_ang = obstacle_info[i+1]/180*Math.PI;
            let o_ang_max = -obstacle_info[i+2]/180*Math.PI;
            let o_ang_min = -obstacle_info[i+3]/180*Math.PI;
            
            let x_= x+o_dis * Math.cos(angle+o_ang);
            let y_= y+o_dis * Math.sin(angle+o_ang);
            arc_dis = Math.sqrt(Math.pow(x-x_,2)+Math.pow(y-y_,2));
            ctx.arc(center_x + (x *0.877), center_y - (y * 0.88), arc_dis, o_ang_max-angle, o_ang_min-angle);
            ctx.stroke();
        }
        ctx.closePath();
        
    }    
}
function draw_move_line(x,y,w,move_angle,move_speed,move_w){
    let canvas = document.getElementById("robot_map");
    let ctx = canvas.getContext("2d");
    let center_x = canvas.width / 2;
    let center_y = canvas.height / 2;
    let angle = (w)/180*Math.PI;
    ctx.lineWidth = 5;
    //=================================
    //console.log(move_angle);
    ctx.beginPath();
    if(move_angle!=999){
        //Y = ( ( X - X1 )( Y2 - Y1) / ( X2 - X1) ) + Y1
        let length = (move_speed-0)*(60-15)/(80-0)+15;
        //console.log(move_speed);
        ctx.strokeStyle = '#3366ff';
        x_= x+length * Math.cos(move_angle+angle);
        y_= y+length * Math.sin(move_angle+angle);
        x_=Math.round(x_);
        y_=Math.round(y_);
        ctx.moveTo(center_x + (x * 0.877), center_y - (y * 0.88));
        ctx.lineTo(center_x + (x_ *0.877), center_y - (y_ * 0.88));
        ctx.stroke();
    }
    ctx.closePath();
    ctx.beginPath();
    let percent = ((Math.abs(move_w)-0)*(100-10)/(50-0)+10)/100;
    if(move_w>0){
        ctx.strokeStyle = '#3366ff';
        ctx.arc(center_x + (x *0.877), center_y - (y * 0.88), 30,  -angle+ Math.PI*(1+(1-percent)), -angle + 2*Math.PI);
        ctx.stroke();
    }
    if(move_w<0){
        ctx.strokeStyle = '#3366ff';
        ctx.arc(center_x + (x *0.877), center_y - (y * 0.88), 30,  -angle, -angle + Math.PI*percent);
        ctx.stroke();
    }
    ctx.closePath();
    //=================================
}
function draw_robot(num,sigma,x,y,w,catchball,imu){
    if(sigma!=null){
        let ground_reverse = document.getElementById("GroundButton").checked;
        let canvas = document.getElementById("robot_map");
        let ctx = canvas.getContext("2d");
        let center_x = canvas.width / 2;
        let center_y = canvas.height / 2;
        let sigma_error = 20;

        let x_imu,y_imu,angle_imu;
        angle_imu = imu;
        x_imu= x+20 * Math.cos(angle_imu);
        y_imu= y+20 * Math.sin(angle_imu);
        x_imu=Math.round(x_imu);
        y_imu=Math.round(y_imu);

        let angle = (w)/180*Math.PI;
        x_= Math.round(x+20 * Math.cos(angle));
        y_= Math.round(y+20 * Math.sin(angle));
        
        tmp_angle = Math.abs((angle-angle_imu)*180/Math.PI);
        if(tmp_angle>180)tmp_angle = 360-tmp_angle;
        //console.log(tmp_angle);
        ctx.beginPath();
        if (sigma < sigma_error && tmp_angle<40) {
            ctx.strokeStyle = '#FFFF00';
            ctx.lineWidth = 5;
        } else {
            ctx.strokeStyle = '#FF0000';
            ctx.lineWidth = 10;
        }
        ctx.arc(center_x + (x *0.877), center_y - (y * 0.88), 19, 0, 2 * Math.PI);
        ctx.moveTo(center_x + (x * 0.877), center_y - (y * 0.88));
        ctx.lineTo(center_x + (x_ *0.877), center_y - (y_ * 0.88));
        ctx.stroke();
        ctx.closePath();
        
        ctx.beginPath();
        ctx.strokeStyle = '#FF0000';
        ctx.lineWidth = 3;
        ctx.moveTo(center_x + (x * 0.877), center_y - (y * 0.88));
        ctx.lineTo(center_x + (x_imu *0.877), center_y - (y_imu * 0.88));
        ctx.stroke();
        ctx.closePath();
        ctx.beginPath();
        ctx.save();
        ctx.font="25px Georgia";
        ctx.fillStyle="black";
        if(ground_reverse==false){
            ctx.translate(center_x+(x*0.877)-8, center_y-(y*0.88)+7);
        }else{
            ctx.translate(center_x+(x*0.877)+5, center_y-(y*0.88)-8);
            ctx.rotate(Math.PI);
        }
        ctx.fillText(num.toString(), 0, 0);
        ctx.restore();
        ctx.closePath();
        if(catchball==1){
          ctx.beginPath();
          ctx.arc(center_x + (x_ * 0.877), center_y - (y_ * 0.88),10,0,360,false);
          ctx.fillStyle="yellow";//填充颜色,默认是黑色
          ctx.fill();//画实心圆
          ctx.closePath();
          ctx.beginPath();
          ctx.strokeStyle = '#000000';
          ctx.arc(center_x + (x_ *0.877), center_y - (y_ * 0.88), 10, 0, 2 * Math.PI);
          ctx.stroke();
          ctx.closePath();
        }
    }
}
function ball_condition(){
  if(catchball2==false&&catchball3==false){
    if(ball_ang1!=999&&ball_ang2==999&&ball_ang3==999){
        draw_ball(x1,y1,w1,ball_ang1,ball_dis1);
    }
    if(ball_ang2==999){
      if(ball_ang3!=999){
        draw_ball(x3,y3,w3,ball_ang3,ball_dis3);
      }
    }
    else if(ball_ang3==999){
      if(ball_ang2!=999){
        draw_ball(x2,y2,w2,ball_ang2,ball_dis2);
      }
    }
    else if(ball_ang2!=999&&ball_ang3!=999){
      let x,y;
      m2=Math.tan((ball_ang2+w2)/180*Math.PI);
      m3=Math.tan((ball_ang3+w3)/180*Math.PI);
      //m2*x-y=m2*x2-y2;
      //m3*x-y=m3*x3-y3;
      let canvas = document.getElementById("robot_map");
      let ctx = canvas.getContext("2d");
      let center_x = canvas.width / 2;
      let center_y = canvas.height / 2;
      x=((m2*x2-y2)-(m3*x3-y3))/(m2-m3);
      y=-(m2*x2-y2-m2*x);
      ctx.arc(center_x + (x * 0.877), center_y - (y * 0.88),13,0,360,false);
      ctx.fillStyle="red";//填充颜色,默认是黑色
      ctx.fill();//画实心圆
      //=======================================
      ctx.beginPath();
      ctx.strokeStyle = '#000000';
      ctx.arc(center_x+(x*0.877), center_y-(y*0.88), 13, 0, 2*Math.PI);
      ctx.stroke();
      ctx.closePath();
    }
    
  }
}
function draw_ball(x,y,w,ball_ang,ball_dis)
{
  let canvas = document.getElementById("robot_map");
  let ctx = canvas.getContext("2d");
  let center_x = canvas.width / 2;
  let center_y = canvas.height / 2;
  ctx.beginPath
  let x_ball,y_ball,angle_ball;
  angle_ball = (ball_ang+w)/180*Math.PI;
  x_ball= x+ball_dis * Math.cos(angle_ball);
  y_ball= y+ball_dis * Math.sin(angle_ball);
  x_ball=Math.round(x_ball);
  y_ball=Math.round(y_ball);
  //ctx.lineWidth = 1;
  //ctx.strokeStyle = '#FFFF00';
  //ctx.moveTo(center_x+(x*1.3), center_y-(y*1.3));
  //ctx.lineTo(center_x+(x_ball*1.3), center_y-(y_ball*1.3));
  ctx.arc(center_x + (x_ball * 0.877), center_y - (y_ball * 0.88),13,0,360,false);
  ctx.fillStyle="red";//填充颜色,默认是黑色
  ctx.fill();//画实心圆
  ctx.closePath();
  ctx.beginPath();
  ctx.strokeStyle = '#000000';
  ctx.arc(center_x+(x_ball*0.877), center_y-(y_ball*0.88), 13, 0, 2*Math.PI);
  ctx.stroke();
  ctx.closePath();
}
var imu_pub1 = new ROSLIB.Topic({
    ros: ros,
    name: 'imu_3d/angle_correction',
    messageType: 'std_msgs/Float32'
});
var imu_pub2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'imu_3d/angle_correction',
    messageType: 'std_msgs/Float32'
});
var imu_pub3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'imu_3d/angle_correction',
    messageType: 'std_msgs/Float32'
});
function ImuReset() {
    let robot_checked1=document.getElementById('RobotOneButton').checked;
    let robot_checked2=document.getElementById('RobotTwoButton').checked;
    let robot_checked3=document.getElementById('RobotThreeButton').checked;
    
    if(robot_checked1){
        var msg = new ROSLIB.Message({
          data: w1-90 
        });
        imu_pub1.publish(msg);
    }
    else if(robot_checked2){
        var msg = new ROSLIB.Message({
          data: w2-90 
        });
        console.log(w2-90);
        imu_pub2.publish(msg);
    }
    else if(robot_checked3){
        var msg = new ROSLIB.Message({
          data: w3-90 
        });
        imu_pub3.publish(msg);
    }
}

//reset
var resetParticles1 = new ROSLIB.Topic({
    ros: ros,
    name: 'mcl/resetParticles',
    messageType: 'self_localization/resetParticles'
});
var resetParticles2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'mcl/resetParticles',
    messageType: 'self_localization/resetParticles'
});
var resetParticles3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'mcl/resetParticles',
    messageType: 'self_localization/resetParticles'
});

function CoordReset() {
    let robot_checked1=document.getElementById('RobotOneButton').checked;
    let robot_checked2=document.getElementById('RobotTwoButton').checked;
    let robot_checked3=document.getElementById('RobotThreeButton').checked;
    
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
    if(robot_checked1){
        resetParticles1.publish(msg);
        reset_bool=false;

    }
    else if(robot_checked2){
        resetParticles2.publish(msg);
        reset_bool=false;
    }
    else if(robot_checked3){
        resetParticles3.publish(msg);
        reset_bool=false;
    }
}
function CoordReverse() {

    let robot_checked1=document.getElementById('RobotOneButton').checked;
    let robot_checked2=document.getElementById('RobotTwoButton').checked;
    let robot_checked3=document.getElementById('RobotThreeButton').checked;
    if(robot_checked1){
        var msg = new ROSLIB.Message({
            init: true,
            x: -x1,
            y: y1,
            w: w1-180
        });
        resetParticles1.publish(msg);
    }
    else if(robot_checked2){
        var msg = new ROSLIB.Message({
            init: true,
            x: -x2,
            y: y2,
            w: w2-180
        });
        resetParticles2.publish(msg);
    }
    else if(robot_checked3){
        var msg = new ROSLIB.Message({
            init: true,
            x: -x3,
            y: y3,
            w: w3-180
        });
        resetParticles3.publish(msg);
    }
    //console.log(msg.w);
    setTimeout(ImuReset,300);
}
/*========================================================*/
/*========================================================*/
//GameState
var GameState1 = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'FIRA/GameState',
    messageType: 'std_msgs/Int32'
});

function PublishTopicGameState(state) {
   // console.log(state);
    var gameState = new ROSLIB.Message({
        data: state
    });
    if (CheckIP[0] == 1)
        GameState1.publish(gameState);
    if (CheckIP[1] == 1)
        GameState2.publish(gameState);
    if (CheckIP[2] == 1)
        GameState3.publish(gameState);
}
/*========================================================*/
//MotionRemote
var Remote1 = new ROSLIB.Topic({
    ros: ros,
    name: 'motion/remote',
    messageType: 'std_msgs/Bool'
});
var Remote2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'motion/remote',
    messageType: 'std_msgs/Bool'
});
var Remote3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'motion/remote',
    messageType: 'std_msgs/Bool'
});

function RemoteSwitch(state) {
    var check;
    if (state) {
       // console.log(123);
        RemoteState = true;
        check = new ROSLIB.Message({
            data: RemoteState
        });
    } else {
        RemoteState = false;
        check = new ROSLIB.Message({
            data: RemoteState
        });
    }
    if (CheckIP[0] == 1)
        Remote1.publish(check);
    if (CheckIP[1] == 1)
        Remote2.publish(check);
    if (CheckIP[2] == 1)
        Remote3.publish(check);
}
/*========================================================*/
//GameState
var GameState1 = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'FIRA/GameState',
    messageType: 'std_msgs/Int32'
});

function PublishTopicGameState(state) {
  //  console.log(state);
    var gameState = new ROSLIB.Message({
        data: state
    });
    if (CheckIP[0] == 1)
        GameState1.publish(gameState);
    if (CheckIP[1] == 1)
        GameState2.publish(gameState);
    if (CheckIP[2] == 1)
        GameState3.publish(gameState);
}
/*========================================================*/
//TeamColor
var TeamColor1 = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
var TeamColor2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
var TeamColor3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'FIRA/TeamColor',
    messageType: '/std_msgs/String'
});

function PublishTopicTeamColor(color) {
    var teamcolor = new ROSLIB.Message({
        data: color
    });

    if (CheckIP[0] == 1)
        TeamColor1.publish(teamcolor);
    if (CheckIP[1] == 1)
        TeamColor2.publish(teamcolor);
    if (CheckIP[2] == 1)
        TeamColor3.publish(teamcolor);
}

/*========================================================*/
//vector
var cmdVel1 = new ROSLIB.Topic({
    ros: ros,
    name: 'motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});

var cmdVel2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
var cmdVel3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});

cmdVel1.subscribe(function(msg) {
    move_angle1 = Math.atan2(-msg.linear.x,msg.linear.y);
    move_speed1 = Math.sqrt(Math.pow(msg.linear.x,2)+Math.pow(msg.linear.y,2));
    if(msg.linear.x==0&&msg.linear.y==0){
        move_angle1=999;
    }
    move_w1 = msg.angular.z;
    //console.log(move_w);
});
cmdVel2.subscribe(function(msg) {
    move_angle2 = Math.atan2(-msg.linear.x,msg.linear.y);
    move_speed2 = Math.sqrt(Math.pow(msg.linear.x,2)+Math.pow(msg.linear.y,2));
    if(msg.linear.x==0&&msg.linear.y==0){
        move_angle2=999;
    }
    move_w2 = msg.angular.z;
    //console.log(move_w);
});
cmdVel3.subscribe(function(msg) {
    move_angle3 = Math.atan2(-msg.linear.x,msg.linear.y);
    move_speed3 = Math.sqrt(Math.pow(msg.linear.x,2)+Math.pow(msg.linear.y,2));
    if(msg.linear.x==0&&msg.linear.y==0){
        move_angle3=999;
    }
    move_w3 = msg.angular.z;
    //console.log(move_w);
});

function StrategyStop() {
    setTimeout(StandBy, 0);
    setTimeout(StandBy, 100);
    setTimeout(StandBy, 200);
    setTimeout(StandBy, 300);
    setTimeout(StandBy, 400);
}

function StandBy() {
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
   // console.log(twist);
    cmdVel1.publish(twist);
    cmdVel2.publish(twist);
    cmdVel3.publish(twist);
}

function PublishTopicCmdVel(vec3) {
   // console.log(vec3);
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
        if (ChooseRobot == 1) {
            cmdVel1.publish(twist);
        } else if (ChooseRobot == 2) {
            cmdVel2.publish(twist);
        } else if (ChooseRobot == 3) {
            cmdVel3.publish(twist);
        }
    }
}

/*========================================================*/
//shoot
var TopicShoot1 = new ROSLIB.Topic({
    ros: ros,
    name: 'motion/shoot',
    messageType: 'std_msgs/Int32'
});
var TopicShoot2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'motion/shoot',
    messageType: 'std_msgs/Int32'
});
var TopicShoot3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'motion/shoot',
    messageType: 'std_msgs/Int32'
});

//shoot
function PublishTopicShoot(size) {
   // console.log(size);
    var Shoot = new ROSLIB.Message({
        data: size
    });
    if (RemoteState) {
        console.log("shoot "+size);
        if (ChooseRobot == 1) {
            TopicShoot1.publish(Shoot);
        } else if (ChooseRobot == 2) {
            TopicShoot2.publish(Shoot);
        } else if (ChooseRobot == 3) {
            TopicShoot3.publish(Shoot);
        }
    }
}
/*========================================================*/
//Vision
var Vision1 = new ROSLIB.Topic({
    ros: ros,
    name: 'vision/object',
    messageType: '/vision/Object'
});
var Vision2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'vision/object',
    messageType: '/vision/Object'
});
var Vision3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'vision/object',
    messageType: '/vision/Object'
});

//Vision
Vision1.subscribe(function(msg) {
    var VBox = [];
    VBox.push(msg.fps);
    VBox.push(msg.ball_dis);
    VBox.push(msg.ball_ang);
    VBox.push(msg.blue_dis);
    VBox.push(msg.blue_ang);
    VBox.push(msg.yellow_dis);
    VBox.push(msg.yellow_ang);
    SaveVision(0, VBox);
    ball_dis1 = msg.ball_dis;
    ball_ang1 = msg.ball_ang;
});
Vision2.subscribe(function(msg) {
    var VBox = [];
    VBox.push(msg.fps);
    VBox.push(msg.ball_dis);
    VBox.push(msg.ball_ang);
    VBox.push(msg.blue_dis);
    VBox.push(msg.blue_ang);
    VBox.push(msg.yellow_dis);
    VBox.push(msg.yellow_ang);
    SaveVision(1, VBox);
    ball_dis2 = msg.ball_dis;
    ball_ang2 = msg.ball_ang;
});
Vision3.subscribe(function(msg) {
    var VBox = [];
    VBox.push(msg.fps);
    VBox.push(msg.ball_dis);
    VBox.push(msg.ball_ang);
    VBox.push(msg.blue_dis);
    VBox.push(msg.blue_ang);
    VBox.push(msg.yellow_dis);
    VBox.push(msg.yellow_ang);
    SaveVision(2, VBox);
    ball_dis3 = msg.ball_dis;
    ball_ang3 = msg.ball_ang;
});

function SaveVision(i, VBox) {
    if ((VisionBox[i].fps != VBox[0]) || (VisionBox[i].ball_dis != VBox[1]) ||
        (VisionBox[i].ball_ang != VBox[2]) || (VisionBox[i].blue_dis != VBox[3]) ||
        (VisionBox[i].blue_ang != VBox[4]) || (VisionBox[i].yellow_dis != VBox[5]) ||
        (VisionBox[i].yellow_ang != VBox[6])) {

        VisionBox[i].fps = VBox[0];
        VisionBox[i].ball_dis = VBox[1];
        VisionBox[i].ball_ang = VBox[2];
        VisionBox[i].blue_dis = VBox[3];
        VisionBox[i].blue_ang = VBox[4];
        VisionBox[i].yellow_dis = VBox[5];
        VisionBox[i].yellow_ang = VBox[6];

        document.getElementsByName('RobotVision' + (i + 1))[0].innerText = VBox[0];
        document.getElementsByName('RobotVision' + (i + 1))[1].innerText = VBox[1] + ',' + VBox[2] + '°';
        document.getElementsByName('RobotVision' + (i + 1))[2].innerText = VBox[3] + ',' + VBox[4] + '°';
        document.getElementsByName('RobotVision' + (i + 1))[3].innerText = VBox[5] + ',' + VBox[6] + '°';
    }
}
/*========================================================*/
//TeamStrategyInformation
//pub
//ros1
var TSInfoPub12 = new ROSLIB.Topic({
    ros: ros2,
    name: 'r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub13 = new ROSLIB.Topic({
    ros: ros3,
    name: 'r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//ros2
var TSInfoPub21 = new ROSLIB.Topic({
    ros: ros,
    name: 'r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub23 = new ROSLIB.Topic({
    ros: ros3,
    name: 'r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//ros3
var TSInfoPub31 = new ROSLIB.Topic({
    ros: ros,
    name: 'r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub32 = new ROSLIB.Topic({
    ros: ros2,
    name: 'r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//subscribe
var TSInfoListen1 = new ROSLIB.Topic({
    ros: ros,
    name: 'r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoListen2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoListen3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});

TSInfoListen1.subscribe(function(msg) {
    var Box = [];
    var item;
    var info;
    for (item = 0; item < 5; item++) {
        Box.push(parseFloat(msg.data[item]));
    }
    catchball1=Box[2]
    info = new ROSLIB.Message({
        data: Box
    });
    if (CheckIP[1])
        TSInfoPub12.publish(info);
    if (CheckIP[2])
        TSInfoPub13.publish(info);
});

TSInfoListen2.subscribe(function(msg) {
    var Box = [];
    var item;
    var info;
    for (item = 0; item < 5; item++) {
        Box.push(parseFloat(msg.data[item]));
    }
    catchball2=Box[2];
    info = new ROSLIB.Message({
        data: Box
    });

    if (CheckIP[0])
        TSInfoPub21.publish(info);
    if (CheckIP[2]) {
        TSInfoPub23.publish(info);
    }
});

TSInfoListen3.subscribe(function(msg) {
    var Box = [];
    var item;
    var info;
    for (item = 0; item < 5; item++) {
        Box.push(parseFloat(msg.data[item]));
    }
    catchball1=Box[2];
    info = new ROSLIB.Message({
        data: Box
    });

    if (CheckIP[0])
        TSInfoPub31.publish(info);
    if (CheckIP[1]) {
        TSInfoPub32.publish(info);
    }
});
/*========================================================*/
//SaveParam 
var SaveParam1 = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});
var SaveParam2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});
var SaveParam3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});

function PublishTopicSaveParam() {
    var SaveParam = new ROSLIB.Message({
        data: 1
    });
    if (CheckIP[0] == 1)
        SaveParam1.publish(SaveParam);
    if (CheckIP[1] == 1)
        SaveParam2.publish(SaveParam);
    if (CheckIP[2] == 1)
        SaveParam3.publish(SaveParam);
  //  console.log("call save");
}
/*========================================================*/
// IsTeamStrategy
var TeamStrategy1 = new ROSLIB.Topic({
    ros: ros,
    name: 'FIRA/IsTeamStrategy',
    messageType: 'std_msgs/Int32'
});
var TeamStrategy2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'FIRA/IsTeamStrategy',
    messageType: 'std_msgs/Int32'
});
var TeamStrategy3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'FIRA/IsTeamStrategy',
    messageType: 'std_msgs/Int32'
});

function IsTeamStrategy(check) {
    //console.log(check);
    var TeamStrategy = new ROSLIB.Message({
        data: parseInt(check)
    });
    if (CheckIP[0] == 1)
        TeamStrategy1.publish(TeamStrategy);
    if (CheckIP[1] == 1)
        TeamStrategy2.publish(TeamStrategy);
    if (CheckIP[2] == 1)
        TeamStrategy3.publish(TeamStrategy);

}
//////////////////////////video/////////////
var View1 = new ROSLIB.Topic({
    ros: ros,
    name: 'vision/view',
    messageType: '/vision/view'
});
var View2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'vision/view',
    messageType: '/vision/view'
});
var View3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'vision/view',
    messageType: '/vision/view'
});

function ViewButton(value) {
   // console.log(value);
    var ViewCheck = new ROSLIB.Message({
        checkpoint: parseInt(value)
    });
    if (ChooseRobot == 1) {
        View1.publish(ViewCheck);
    } else if (ChooseRobot == 2) {
        View2.publish(ViewCheck);
    } else if (ChooseRobot == 3) {
        View3.publish(ViewCheck);
    }

}
//=====================================================================================
// hold ball
var HoldBall1 = new ROSLIB.Topic({
    ros: ros,
    name: 'motion/hold_ball',
    messageType: 'std_msgs/Bool'
});
var HoldBall2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'motion/hold_ball',
    messageType: 'std_msgs/Bool'
});
var HoldBall3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'motion/hold_ball',
    messageType: 'std_msgs/Bool'
});

function HoldBallSwitch(state,robot) {
    var check;
    if (state) {
       // console.log(robot,"hold ball :",state);
        check = new ROSLIB.Message({
            data: true
        });
    } else {
       // console.log(robot,"hold ball :",state);
        check = new ROSLIB.Message({
            data: false
        });
    }
    if (CheckIP[0] == 1 && robot == 1)
        HoldBall1.publish(check);
    if (CheckIP[1] == 1 && robot == 2)
        HoldBall2.publish(check);
    if (CheckIP[2] == 1 && robot == 3)
        HoldBall3.publish(check);
}
//=========================================
var obstacle1 = new ROSLIB.Topic({
    ros: ros,
    name: 'vision/obstacle',
    messageType: '/std_msgs/Int32MultiArray'
});
var obstacle2 = new ROSLIB.Topic({
    ros: ros2,
    name: 'vision/obstacle',
    messageType: '/std_msgs/Int32MultiArray'
});
var obstacle3 = new ROSLIB.Topic({
    ros: ros3,
    name: 'vision/obstacle',
    messageType: '/std_msgs/Int32MultiArray'
});

obstacle1.subscribe(function(msg) {
   obstacle_info1 = msg.data;
});
obstacle2.subscribe(function(msg) {
   obstacle_info2 = msg.data;
});
obstacle3.subscribe(function(msg) {
   obstacle_info3 = msg.data;
   //console.log(obstacle_info3);
});