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
    name: '/motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
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
    let str;
    //str="---"+"<br/>"+"linear:<br/>"+"&ensp;&ensp;x: "+String(Math.round(vec3.x*100)/100)+"<br/>"+"&ensp;&ensp;y: "+String(Math.round(vec3.y*100)/100)+"<br/>"+"&ensp;&ensp;z: 0.0<br/>";
    //str=str+"angular:<br/>"+"&ensp;&ensp;x: 0.0"+"<br/>"+"&ensp;&ensp;y: 0.0"+"<br/>"+"&ensp;&ensp;z: "+String(Math.round(vec3.z*100)/100);
    if (RemoteState) {
      console.log(twist);
      //SendMsgs(str,0);
      cmdVel1.publish(twist);
    }
}
