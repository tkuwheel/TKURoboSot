//shoot
var TopicShoot1 = new ROSLIB.Topic({
    ros: ros,
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
        TopicShoot1.publish(Shoot);
    }
}
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
    state=document.getElementById("StandbyButton").checked;
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

//cmd_vel
var cmdVel1 = new ROSLIB.Topic({
    ros: ros,
    name: localStorage.SPEED_TOPIC,
    messageType: '/geometry_msgs/Twist'
});
function reset_cmd()
{
    cmdVel1 = new ROSLIB.Topic({
        ros: ros,
        name: localStorage.SPEED_TOPIC,
        messageType: '/geometry_msgs/Twist'
    });
}
function topicSaveParam(value){
    console.log(value);
    var Param = new ROSLIB.Message({
        data:value
    })
    SaveParam.publish(Param);
}
function PublishTopicCmdVel(vec3) {
    let str;
    let x_,y_,z_;
    //console.log(localStorage.COORD)
    if(localStorage.COORD==1){
        x_ = vec3.x;
        y_ = vec3.y;
        z_ = vec3.z;
    }
    else{
        x_ = vec3.y;
        y_ = -vec3.x;
        z_ = vec3.z;
    }
    str="---"+"<br/>"+"linear:<br/>"+"&ensp;&ensp;x: "+ num2str(x_) +"<br/>"+"&ensp;&ensp;y: "+ num2str(y_)+"<br/>"+"&ensp;&ensp;z: 0.0<br/>";
    str=str+"angular:<br/>"+"&ensp;&ensp;x: 0.0"+"<br/>"+"&ensp;&ensp;y: 0.0"+"<br/>"+"&ensp;&ensp;z: "+ num2str(z_) + "<br/>---";
        var twist = new ROSLIB.Message({
        linear: {
            x: x_,
            y: y_,
            z: 0
        },
        angular: {
            x: 0,
            y: 0,
            z: z_
        }
    });
    if (RemoteState) {
      //console.log(twist);
      show_vel(str);
      //SendMsgs(str,0);
      cmdVel1.publish(twist);
    }
}
function num2str(value){
    value = parseFloat(Math.round(value*100)/100);
    let str;    
    if((value*10)%10==0){
        str = String(value)+".0";
    }else{
        str = String(value);
    }
    return str;
}

