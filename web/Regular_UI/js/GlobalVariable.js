//ROS_Connect
var CheckIP = [0, 0, 0];
var RobNum = [0, 1, 2];

//Ros_CheckParam
var CheckGetParm = 0;

//Arbitrary
var obj;

//Choose_Robot
var ChooseRobot = 2;

//Remote_state
var RemoteState = 0;

//Joystick
var boom = new Image();
boom.src = 'img/boom.png';
boom.onload = function() {
    joy_ctx.drawImage(boom, 0, 0);
}
var joystick_Area = document.getElementById('Joystick');
var joystick_canvas = document.getElementById("Joystick_Canvas");
var joy_ctx = joystick_canvas.getContext("2d");
var Round_r = 105; //var Round_r = 60;
var mouse_click = 0;
var logButton = -1;
var windowWidth = window.innerWidth;
var windowHeight = window.innerHeight;
var windowWidthToHeight = windowWidth / windowHeight;
var joystickcenter = {
    x: joystick_Area.offsetWidth / 2,
    y: joystick_Area.offsetHeight / 2
};
var joystick_V = {
    x: 0,
    y: 0,
    ang: 0
};

//vision information
var VisionBox = [];
VisionBox[0] = {
    fps: -999,
    ball_dis: -999,
    ball_ang: -999,
    blue_dis: -999,
    blue_ang: -999,
    yellow_dis: -999,
    yellow_ang: -999
};
VisionBox[1] = {
    fps: -999,
    ball_dis: -999,
    ball_ang: -999,
    blue_dis: -999,
    blue_ang: -999,
    yellow_dis: -999,
    yellow_ang: -999
};
VisionBox[2] = {
    fps: -999,
    ball_dis: -999,
    ball_ang: -999,
    blue_dis: -999,
    blue_ang: -999,
    yellow_dis: -999,
    yellow_ang: -999
};


//Robot show
var ShowFlag = 1;
var ShowTime = 0;