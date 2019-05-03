window.addEventListener("keydown", keysdown, false);
window.addEventListener("keyup", keyuped, false);

var keys = [];
var start;
var vec3;

function KeyboardState(state) {
    start = state;
}

function keysdown(e) {
    if (start == true) {
        var speed = document.getElementById("SpeedInput").value;
        keys[e.keyCode] = true;

        //Strategy_Choose
        if (keys[32] && keys[49]) {
            SetBehaviorKeyborard([0, 1, 1, 0, 0, 0, 0]);
            e.preventDefault();
        } else if (keys[32] && keys[50]) {
            SetBehaviorKeyborard([1, 0, 1, 0, 0, 0, 0]);
            e.preventDefault();
        } else if (keys[32] && keys[51]) {
            SetBehaviorKeyborard([0, 1, 0, 0, 1, 0, 0]);
            e.preventDefault();
        } else if (keys[32] && keys[52]) {
            SetBehaviorKeyborard([1, 0, 0, 0, 1, 0, 0]);
            e.preventDefault();
        } else if (keys[32] && keys[53]) {
            SetBehaviorKeyborard([0, 1, 0, 0, 0, 1, 0]);
            e.preventDefault();
        } else if (keys[32] && keys[54]) {
            SetBehaviorKeyborard([1, 0, 0, 0, 0, 1, 0]);
            e.preventDefault();
        } else if (keys[32] && keys[55]) {
            SetBehaviorKeyborard([0, 1, 0, 1, 0, 0, 0]);
            e.preventDefault();
        } else if (keys[32] && keys[56]) {
            SetBehaviorKeyborard([1, 0, 0, 1, 0, 0, 0]);
            e.preventDefault();
        }
        //RobotControl
        if (keys[87] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[87] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[83] && keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[83] && keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed / Math.pow(2, 0.5)),
                y: -parseFloat(speed / Math.pow(2, 0.5)),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[87]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: parseFloat(speed),
                z: 0
            });
            PublishTopicCmdVel(vec3);
           // PublishTopicCmdVel(vec3);
        } else if (keys[68]) {
            vec3 = new ROSLIB.Message({
                x: parseFloat(speed),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[83]) {
            vec3 = new ROSLIB.Message({
                x: 0,
                y: -parseFloat(speed),
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[65]) {
            vec3 = new ROSLIB.Message({
                x: -parseFloat(speed),
                y: 0,
                z: 0
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[69]) {
            //if (speed > 30)
            //    speed = speed * 0.5;
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: -parseFloat(speed)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[81]) {
            //if (speed > 30)
            //    speed = speed * 0.5;
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: parseFloat(speed)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        }
        //SwitchRobot
        if (keys[80]) {
            PublishTopicGameState(0);
            StrategyStop();
        } else if (keys[79]) {
            PublishTopicGameState(1);
        }

    }
}

function releasebutton(state) {
    let vec3_ = new ROSLIB.Message({
        x: 0,
        y: 0,
        z: 0
    });
    switch(state){
      case 81:
        vec3.z = 0;
        break;
      case 69:
        vec3.z = 0;
        break;
      case 87:
        vec3.y = 0;
        break;
      case 65:
        vec3.x = 0;
        break;
      case 83:
        vec3.y = 0;
        break;
      case 68:
        vec3.x = 0;
        break;
      default:
        vec3.x = 0;
        vec3.y = 0;
        vec3.Z = 0;
    }
    PublishTopicCmdVel(vec3);
    //PublishTopicCmdVel(vec3);
}

function keyuped(e) {
    if (start) {
      console.log(1111);
        if (keys[e.keyCode] == true) releasebutton(e.keyCode);
        //else if (keys[69] == true) releasebutton(69);
        //else if (keys[87] == true) releasebutton(87);
        //else if (keys[65] == true) releasebutton(65);
        //else if (keys[83] == true) releasebutton(83);
        //else if (keys[68] == true) releasebutton(68);
        keys[e.keyCode] = false;
    }
}
