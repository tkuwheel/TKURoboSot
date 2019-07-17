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

        // m imu reset
        if (keys[77]) {
            ImuReset();
        }
       
        // , hold ball
        if (keys[188]) {
            //console.log(e.keyCode);
            let hold_ball_checked = document.getElementById("HoldBallButton1").checked;
            //console.log(hold_ball_checked);

           if (hold_ball_checked) {
                $("#HoldBallButton1").remove();
                var holdball_i = $(document.createElement('i'))
                    .attr("class", "fa fa-futbol-o fa-2x")
                    .attr("aria-hidden", "true")
                    .attr("id", "HoldBallButton1")
                holdball_i.appendTo("#HoldBallDiv1");
                this.value = 0;
                HoldBallSwitch(0,1);
                $('#HoldBallButton1').prop('checked',false);
                $('#HoldBallButton1').change();
            } else {
                $("#HoldBallButton1").remove();
                var holdball_i = $(document.createElement('i'))
                    .attr("class", "fa fa-futbol-o fa-2x fa-spin")
                    .attr("aria-hidden", "true")
                    .attr("id", "HoldBallButton1")
                    .attr("style","color: #FFCC00;")
                holdball_i.appendTo("#HoldBallDiv1");
                this.value = 1;
                HoldBallSwitch(1,1);
                $('#HoldBallButton1').prop('checked',true);
                $('#HoldBallButton1').change();
            }
        }
        // . start
        if (keys[190]) {
            topicROSGameState(9);
        }
        // / stop
        if (keys[191]) {
            topicROSGameState(0);
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
            var speed_;
            if (Math.abs(parseFloat(speed)) > 15){
              speed_ = parseFloat(speed) * 0.6;
            }else{
              speed_ = speed;
            }
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: -parseFloat(speed_)
            });
            PublishTopicCmdVel(vec3);
            //PublishTopicCmdVel(vec3);
        } else if (keys[81]) {
            var speed_;
            if (Math.abs(parseFloat(speed)) > 15){
              speed_ = parseFloat(speed) * 0.6;
            }else{
              speed_ = speed;
            }
            vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: parseFloat(speed_)
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
    let vec3 = new ROSLIB.Message({
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
    //if(state==81||state==69||state==87||state==65||state==83||state==68){
    //    console.log("stop");
    //    PublishTopicCmdVel(vec3);
    //}
    console.log("stop");
    PublishTopicCmdVel(vec3);
}

function keyuped(e) {
    if (start) {
        //console.log(1111);
        if (keys[e.keyCode] == true) releasebutton(e.keyCode);
        //else if (keys[69] == true) releasebutton(69);
        //else if (keys[87] == true) releasebutton(87);
        //else if (keys[65] == true) releasebutton(65);
        //else if (keys[83] == true) releasebutton(83);
        //else if (keys[68] == true) releasebutton(68);
        keys[e.keyCode] = false;
    }
}
