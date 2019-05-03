/*========================================================*/
//MotionRemote
var Remote1 = new ROSLIB.Topic({
    ros: ros,
    name: '/motion/remote',
    messageType: 'std_msgs/Bool'
});
var Remote2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/motion/remote',
    messageType: 'std_msgs/Bool'
});
var Remote3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/motion/remote',
    messageType: 'std_msgs/Bool'
});

function RemoteSwitch(state) {
    var check;
    if (state) {
        console.log(123);
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
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});
var GameState3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/FIRA/GameState',
    messageType: 'std_msgs/Int32'
});

function PublishTopicGameState(state) {
    console.log(state);
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
    name: '/FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
var TeamColor2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/FIRA/TeamColor',
    messageType: '/std_msgs/String'
});
var TeamColor3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/FIRA/TeamColor',
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
    name: '/motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});

var cmdVel2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
});
var cmdVel3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/motion/cmd_vel',
    messageType: '/geometry_msgs/Twist'
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
    console.log(twist);
    cmdVel1.publish(twist);
    cmdVel2.publish(twist);
    cmdVel3.publish(twist);
}

function PublishTopicCmdVel(vec3) {
    console.log(vec3);
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
    name: '/motion/shoot',
    messageType: 'std_msgs/Int32'
});
var TopicShoot2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/motion/shoot',
    messageType: 'std_msgs/Int32'
});
var TopicShoot3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/motion/shoot',
    messageType: 'std_msgs/Int32'
});

//shoot
function PublishTopicShoot(size) {
    console.log(size);
    var Shoot = new ROSLIB.Message({
        data: size
    });
    if (RemoteState) {
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
    name: '/vision/object',
    messageType: '/vision/Object'
});
var Vision2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/vision/object',
    messageType: '/vision/Object'
});
var Vision3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/vision/object',
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
    name: '/r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub13 = new ROSLIB.Topic({
    ros: ros3,
    name: '/r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//ros2
var TSInfoPub21 = new ROSLIB.Topic({
    ros: ros,
    name: '/r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub23 = new ROSLIB.Topic({
    ros: ros3,
    name: '/r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//ros3
var TSInfoPub31 = new ROSLIB.Topic({
    ros: ros,
    name: '/r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoPub32 = new ROSLIB.Topic({
    ros: ros2,
    name: '/r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});
//subscribe
var TSInfoListen1 = new ROSLIB.Topic({
    ros: ros,
    name: '/r1_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoListen2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/r2_info',
    messageType: '/std_msgs/Float32MultiArray'
});
var TSInfoListen3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/r3_info',
    messageType: '/std_msgs/Float32MultiArray'
});

TSInfoListen1.subscribe(function(msg) {
    var Box = [];
    var item;
    var info;
    for (item = 0; item < 5; item++) {
        Box.push(parseFloat(msg.data[item]));
    }
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
    name: '/FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});
var SaveParam2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/FIRA/SaveParam',
    messageType: 'std_msgs/Int32'
});
var SaveParam3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/FIRA/SaveParam',
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
    console.log("call save");
}
/*========================================================*/
// IsTeamStrategy
var TeamStrategy1 = new ROSLIB.Topic({
    ros: ros,
    name: '/FIRA/IsTeamStrategy',
    messageType: 'std_msgs/Int32'
});
var TeamStrategy2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/FIRA/IsTeamStrategy',
    messageType: 'std_msgs/Int32'
});
var TeamStrategy3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/FIRA/IsTeamStrategy',
    messageType: 'std_msgs/Int32'
});

function IsTeamStrategy(check) {
    console.log(check);
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
    name: '/vision/view',
    messageType: '/vision/view'
});
var View2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/vision/view',
    messageType: '/vision/view'
});
var View3 = new ROSLIB.Topic({
    ros: ros3,
    name: '/vision/view',
    messageType: '/vision/view'
});

function ViewButton(value) {
    console.log(value);
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
    name: '/motion/hold_ball',
    messageType: 'std_msgs/Bool'
});
var HoldBall2 = new ROSLIB.Topic({
    ros: ros2,
    name: '/motion/hold_ball',
    messageType: 'std_msgs/Bool'
});
var HoldBall3 = new ROSLIB.Topic({
    ros: ros3,
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
    if (CheckIP[0] == 1 && robot == 1)
        HoldBall1.publish(check);
    if (CheckIP[1] == 1 && robot == 2)
        HoldBall2.publish(check);
    if (CheckIP[2] == 1 && robot == 3)
        HoldBall3.publish(check);
}