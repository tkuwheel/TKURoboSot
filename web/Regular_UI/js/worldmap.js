var worldmap_ctx = worldmap_canvas.getContext("2d");
var window_scale;

worldmap_img.onload = function() {
    Str_Area();
}
worldmap_img.src = 'img/Ground.png';


function Str_Area() {
    windowWidth = window.innerWidth;
    windowHeight = window.innerHeight;
    windowWidthToHeight = windowWidth / windowHeight;

    if (windowWidthToHeight > widthToheight) {
        windowWidth = windowHeight * widthToheight;
        strategy_Area.style.height = windowHeight + 'px';
        strategy_Area.style.width = windowWidth + 'px';
    } else {
        windowHeight = windowWidth / widthToheight;
        strategy_Area.style.width = windowWidth + 'px';
        strategy_Area.style.height = windowHeight + 'px';
    }

    window_scale = windowWidth / 750;

    strategy_Area.style.marginTop = (-windowHeight / 2) + 'px';
    strategy_Area.style.marginLeft = (-windowWidth / 2) + 'px';

    worldmap_canvas.width = windowWidth;
    worldmap_canvas.height = windowWidth / worldmap.widthToheight;
    worldmap_ctx.drawImage(worldmap_img, 0, 0, windowWidth, windowWidth / worldmap.widthToheight);

    /*	function joystick_main(){
    		var ctx = joystick_Area.getContext("2d");
    		var cty = joystick_Area.getContext("2d");

    		ctx.moveTo1(75,75);
    		ctx.lineTo(150,75);
    		ctx.strokeStyle="#0000ff";
    	};*/
    joystick_main();
    // drawRobot();
    //world_location(23.5,23.5);
}

window.addEventListener('resize', Str_Area, false);
window.addEventListener('orientationchange', Str_Area, false);

/*worldmap_canvas.addEventListener("mousedown", function(e){
	var pos = getMousePos(worldmap_canvas, e);
	//alert("x:" + pos.x + " y:" + pos.y);
	console.log(worldmap_canvas.offsetWidth,worldmap_canvas.offsetHeight);
	console.log(pos.x ,pos.y);
	drawRobot2(pos.x,pos.y,22.5,90,0);
})*/
/*worldmap_canvas.addEventListener("mouseup", function(e){
	worldmap_ctx.clearRect(0,0,windowWidth,windowHeight);
	Str_Area();
})*/
var Robot_1 = {
    x: 23.5,
    y: 23.5,
    r: 22.5,
    R: 45,
    rad: 0
};

function world_location() {
    //worldmap_ctx.clearRect(0,0,windowWidth,windowHeight);
    //Str_Area();
    var x = 10 * window_scale;
    var y = 25 * window_scale;

    worldmap_ctx.textAlign = "start";
    worldmap_ctx.font = 15 * window_scale + "px Lucida Console";
    worldmap_ctx.fillStyle = "black";
    worldmap_ctx.fillText("Robot_1 X : " + RobposX[0] + " Y : " + RobposY[0], x, y);
    worldmap_ctx.fillText("Robot_2 X : " + RobposX[1] + " Y : " + RobposY[1], x, y + 20 * window_scale);
    worldmap_ctx.fillText("Robot_3 X : " + RobposX[2] + " Y : " + RobposY[2], x, y + 40 * window_scale);
}

function world_Vision() {
    worldmap_ctx.clearRect(0, 0, windowWidth, windowHeight);
    Str_Area();
    world_location();
    var x = 10 * window_scale;
    var y = 490 * window_scale;

    worldmap_ctx.textAlign = "start";
    worldmap_ctx.font = 10 * window_scale + "px Lucida Console";
    worldmap_ctx.fillStyle = "blue";
    // worldmap_ctx.fillText("Robot_1 fps : " + VisionBox[0].fps, x, y);
    // worldmap_ctx.fillStyle = "rgb(204,0,0)";
    // worldmap_ctx.fillText("Ball_dis : " + VisionBox[0].ball_dis + " Ball_ang : " + VisionBox[0].ball_ang, x + 100* window_scale, y);
    // worldmap_ctx.fillStyle = "blue";
    // worldmap_ctx.fillText("Blue_dis : " + VisionBox[0].blue_dis + " Blue_ang : " + VisionBox[0].blue_ang, x + 260* window_scale, y);
    // worldmap_ctx.fillStyle = "rgb(255,255,0)";
    // worldmap_ctx.fillText("Yellow_dis : " + VisionBox[0].yellow_dis + " Yellow_ang : " + VisionBox[0].yellow_ang, x + 425* window_scale, y);

    // worldmap_ctx.fillStyle = "black";
    // worldmap_ctx.fillText("Robot_2 fps : " + VisionBox[1].fps, x, y + 20 * window_scale);
    // worldmap_ctx.fillStyle = "rgb(204,0,0)";
    // worldmap_ctx.fillText("Ball_dis : " + VisionBox[1].ball_dis + " Ball_ang : " + VisionBox[1].ball_ang, x + 100* window_scale, y + 20 * window_scale);
    // worldmap_ctx.fillStyle = "blue";
    // worldmap_ctx.fillText("Blue_dis : " + VisionBox[1].blue_dis + " Blue_ang : " + VisionBox[1].blue_ang, x + 260* window_scale, y + 20 * window_scale);
    // worldmap_ctx.fillStyle = "rgb(255,255,0)";
    // worldmap_ctx.fillText("Yellow_dis : " + VisionBox[1].yellow_dis + " Yellow_ang : " + VisionBox[1].yellow_ang, x + 425* window_scale, y + 20 * window_scale);

    // worldmap_ctx.fillStyle = "black";
    // worldmap_ctx.fillText("Robot_3 fps : " + VisionBox[2].fps, x, y + 40 * window_scale);
    // worldmap_ctx.fillStyle = "rgb(204,0,0)";
    // worldmap_ctx.fillText("Ball_dis : " + VisionBox[2].ball_dis + " Ball_ang : " + VisionBox[2].ball_ang, x + 100* window_scale, y + 40 * window_scale);
    // worldmap_ctx.fillStyle = "blue";
    // worldmap_ctx.fillText("Blue_dis : " + VisionBox[2].blue_dis + " Blue_ang : " + VisionBox[2].blue_ang, x + 260* window_scale, y + 40 * window_scale);
    // worldmap_ctx.fillStyle = "rgb(255,255,0)";
    // worldmap_ctx.fillText("Yellow_dis : " + VisionBox[2].yellow_dis + " Yellow_ang : " + VisionBox[2].yellow_ang, x + 425* window_scale, y + 40 * window_scale);

    worldmap_ctx.fillText("Robot_1 fps : " + VisionBox[0].fps + " Ball : " + VisionBox[0].ball_ang + " " + VisionBox[0].ball_dis + " Blue : " + VisionBox[0].blue_ang + " " + VisionBox[0].blue_dis + " Yellow : " + VisionBox[0].yellow_ang + " " + VisionBox[0].yellow_dis, x, y - 5);
    worldmap_ctx.fillStyle = "rgb(204,0,0)";
    worldmap_ctx.fillText("Robot_2 fps : " + VisionBox[1].fps + " Ball : " + VisionBox[1].ball_ang + " " + VisionBox[1].ball_dis + " Blue : " + VisionBox[1].blue_ang + " " + VisionBox[1].blue_dis + " Yellow : " + VisionBox[1].yellow_ang + " " + VisionBox[1].yellow_dis, x, y + 15 * window_scale);
    worldmap_ctx.fillStyle = "rgb(255,255,0)";
    worldmap_ctx.fillText("Robot_3 fps : " + VisionBox[2].fps + " Ball : " + VisionBox[2].ball_ang + " " + VisionBox[2].ball_dis + " Blue : " + VisionBox[2].blue_ang + " " + VisionBox[2].blue_dis + " Yellow : " + VisionBox[2].yellow_ang + " " + VisionBox[2].yellow_dis, x, y + 35 * window_scale);
}

function drawRobot2(x, y, r, R, rad, number) {
    x = (x + 375) * window_scale;
    y = (y + 275) * window_scale;
    r = r * window_scale;

    // x = x+375;
    // y = y+275;

    worldmap_ctx.beginPath();
    worldmap_ctx.strokeStyle = "black";
    worldmap_ctx.arc(x, y, r, 0, 2 * Math.PI);
    worldmap_ctx.lineWidth = 1;
    worldmap_ctx.stroke();
    worldmap_ctx.fillStyle = "red";
    worldmap_ctx.globalAlpha = 0.7;
    worldmap_ctx.fill();

    worldmap_ctx.beginPath();
    worldmap_ctx.strokeStyle = "blue";
    worldmap_ctx.moveTo(x, y);
    worldmap_ctx.lineTo(x + r * Math.cos(rad), y + r * Math.sin(rad));
    worldmap_ctx.stroke();
    worldmap_ctx.strokeText(number, x, y);
}