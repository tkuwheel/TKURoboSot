//====================================

var OldCounter = 0;
// ===================================
//
var light;
var canvas = document.getElementById("Light");
var context = canvas.getContext("2d");
context.font = '30pt Calibri';
context.arc(20, 20, 20, 0, Math.PI * 2, false);
//====================================

var OrangeBox = [];
var GreenBox = [];
var BlueBox = [];
var YellowBox = [];
var WhiteBox = [];
var RedconeBox = [];

//=======================================
//var ButtonFlag = 0;
var ColorButton = 4;
//=======================================
var video_canvas = document.getElementById("playerfunction");
//=======================================
websocket_server = false;
websocket_server_msg = "";
connect_request = false;
