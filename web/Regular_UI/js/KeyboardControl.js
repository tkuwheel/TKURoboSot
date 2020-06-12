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
        //==============
        //RobotControl
        vec3 = new ROSLIB.Message({
            x: 0,
            y: 0,
            z: 0
        });

        if (keys[87]) {//w
            vec3.y = parseFloat(speed);
        }
        if (keys[68]) {//d
            vec3.x = parseFloat(speed);
        }
        if (keys[83]) {//s
            vec3.y = -parseFloat(speed);
        }
        if (keys[65]) {//a
            vec3.x = -parseFloat(speed);
        }
        if (keys[69]) {//e
            let speed_;
            if (Math.abs(parseFloat(speed)) > 15) {
                speed_ = parseFloat(speed) * 0.5;
            } else {
                speed_ = speed;
            }
            vec3.z = -parseFloat(speed_);
        }
        if (keys[81]) {//q
            let speed_;
            if (Math.abs(parseFloat(speed)) > 15) {
                speed_ = parseFloat(speed) * 0.5;
            } else {
                speed_ = speed;
            }
            vec3.z = parseFloat(speed_);
        }
        if (keys[87] && keys[68]) {
            vec3.x = parseFloat(speed / Math.pow(2, 0.5));
            vec3.y = parseFloat(speed / Math.pow(2, 0.5));
        }
        if (keys[87] && keys[65]) {
            vec3.x = -parseFloat(speed / Math.pow(2, 0.5));
            vec3.y = parseFloat(speed / Math.pow(2, 0.5));
        }
        if (keys[83] && keys[68]) {
            vec3.x = parseFloat(speed / Math.pow(2, 0.5));
            vec3.y = -parseFloat(speed / Math.pow(2, 0.5));
        }
        if (keys[83] && keys[65]) {
            vec3.x = -parseFloat(speed / Math.pow(2, 0.5));
            vec3.y = -parseFloat(speed / Math.pow(2, 0.5));
        }
        PublishTopicCmdVel(vec3);
        //==============
        //shoot key space
        if (keys[74]) {
            PublishTopicShoot(parseInt(document.getElementById('ShootInput').value));
        }
        if (keys[73]) {
            if (RemoteState) {
                if (ChooseRobot == 1) {
                    holdball(1);
                } else if (ChooseRobot == 2) {
                    holdball(2);
                } else if (ChooseRobot == 3) {
                    holdball(3);
                }
            }
        }
        //SwitchRobot
        // P stop
        if (keys[80]) {
            PublishTopicGameState(0);
            StrategyStop();
            $('#StartInput').prop('checked', false);
            $('#StartInput').change();
            $('#StopInput').prop('checked', true);
            $('#StopInput').change();
            all_stop();
        }
        // O start 
        else if (keys[79]) {
            PublishTopicGameState(1);
            $('#StartInput').prop('checked', true);
            $('#StartInput').change();
            $('#StopInput').prop('checked', false);
            $('#StopInput').change();
            //start_state();
            all_start();
        }

    }
}

function releasebutton(state) {
    let vec3 = new ROSLIB.Message({
        x: 0,
        y: 0,
        z: 0
    });
    switch (state) {
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
    //if(state==81||state==69||state==87||state==65||state==83||state==68){
    //    console.log("stop");
    //    PublishTopicCmdVel(vec3);
    //}
    console.log("stop");
    PublishTopicCmdVel(vec3);
}

function keyuped(e) {
    if (start) {
        //console.log("start moving");
        if (keys[e.keyCode] == true) releasebutton(e.keyCode);
        //else if (keys[69] == true) releasebutton(69);
        //else if (keys[87] == true) releasebutton(87);
        //else if (keys[65] == true) releasebutton(65);
        //else if (keys[83] == true) releasebutton(83);
        //else if (keys[68] == true) releasebutton(68);
        keys[e.keyCode] = false;
    }
}
