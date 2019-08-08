var joy_ctx = joystick_canvas.getContext("2d");
var border_ctx = joystick_canvas.getContext("2d");
var border_cty = joystick_canvas.getContext("2d");
var Round_r = 100; //var Round_r = 60;
var mouse_click = 0;


function drawjoystick(x , y){
	drawjoystick_base();
	joy_ctx.beginPath();
	joy_ctx.arc(x,y,8,0,2*Math.PI);
	joy_ctx.globalAlpha=1;
	joy_ctx.lineWidth = 16;
	joy_ctx.stroke();

	//console.log((joystick_V.x/120)*speed,(joystick_V.y/120)*speed);
    var vec3 = new ROSLIB.Message({
		x : (joystick_V.x/120)*speed,
		y : (joystick_V.y/120)*speed,
		z : 0
	});
	// ROStransfer('joystick',box,button,vec3);
	topicROStransfer(robotChoice,vec3);
	//console.log((joystick_V.x/120)*speed,(joystick_V.y/120)*speed);
	// topic_cmdVel_pub(joystick_V.x/60,joystick_V.y/60,0);
}

function drawjoystick_base(){
	joy_ctx.beginPath();
	joy_ctx.arc(joystick_canvas.width/2,joystick_canvas.height/2,Round_r,0,2*Math.PI);
	joy_ctx.globalAlpha=0.7;


	var grd=worldmap_ctx.createRadialGradient(joystick_canvas.width/2,joystick_canvas.height/2,0,joystick_canvas.width/2,joystick_canvas.height/2,Round_r+5);
	grd.addColorStop(0,"rgba(255,255,255,1)");
	grd.addColorStop(0.2,"rgba(255,255,255,1)");
	grd.addColorStop(0.4,"rgba(255,255,255,1)");
	grd.addColorStop(0.6,"rgba(255,255,255,1)");
	grd.addColorStop(0.8,"rgba(255,255,255,1)");
	grd.addColorStop(0.9,"rgba(255,255,255,1)");
	grd.addColorStop(0.95,"rgba(255,255,255,1)");
	grd.addColorStop(1,"rgba(255,255,255,1)");
	joy_ctx.strokeStyle = grd;
	joy_ctx.lineWidth = 1;
	joy_ctx.stroke();

/*	var grd=worldmap_ctx.createRadialGradient(joystick_canvas.width/2,joystick_canvas.height/2,0,joystick_canvas.width/2,joystick_canvas.height/2,Round_r+5);
	grd.addColorStop(0,"White");
	grd.addColorStop(0.2,"Blue");
	grd.addColorStop(0.4,"Green");
	grd.addColorStop(0.6,"Yellow");
	grd.addColorStop(0.8,"Orange");
	grd.addColorStop(0.9,"Red");
	grd.addColorStop(0.95,"White");
	grd.addColorStop(1,"Black");
	joy_ctx.strokeStyle = grd;
	joy_ctx.lineWidth = 8;
	joy_ctx.stroke();*/

}


var joystickcenter = {
	x : joystick_Area.offsetWidth/2,
	y : joystick_Area.offsetHeight/2
};

var joystick_V = {
	x : 0,
	y : 0,
	ang : 0
};

function getMousePos (cv, e){
	var rect = cv.getBoundingClientRect();
	return {
		x: e.clientX - rect.left,//相對於Canvas左上角的X座標
		y: e.clientY - rect.top,//相對於Canvas左上角的Y座標
		rectLeft : rect.left,
		rectTop : rect.top,
		clientX : e.clientX,
		clientY : e.clientY
	}
}

function Pos_Vector(_pos){
	joystick_V.x = _pos.x - joystickcenter.x;
	joystick_V.y = joystickcenter.y - _pos.y;
	joystick_V.ang = Math.atan2(joystick_V.y,joystick_V.x);
}

function joystick_main(){
	joystick_canvas.width = joystick_Area.offsetWidth;
	joystick_canvas.height = joystick_Area.offsetHeight;
	drawjoystick(joystickcenter.x,joystickcenter.y);
}

function checkArea(){
	var line = Math.sqrt(Math.pow(joystick_V.x,2) + Math.pow(joystick_V.y,2));
	if(line > 120){
		return 0;
	}else{
		return 1;
	}
}

joystick_canvas.addEventListener("mouseenter", function(e){
	var pos = getMousePos(joystick_canvas, e);
	Pos_Vector(pos);
})

joystick_canvas.addEventListener("mousedown", function(e){
	var pos = getMousePos(joystick_canvas, e);
	Pos_Vector(pos);
	
	if(checkArea() == 1){
		joy_ctx.clearRect(0,0,windowWidth,windowHeight);
		drawjoystick(pos.x,pos.y)
		mouse_click = 1;
	}
})

joystick_canvas.addEventListener("mousemove", function(e){
	var pos = getMousePos(joystick_canvas, e);
	Pos_Vector(pos);
	if(mouse_click == 1){
		if(checkArea()){
			joy_ctx.clearRect(0,0,windowWidth,windowHeight);
			drawjoystick(pos.x,pos.y);
		}else{
			pos.x = Round_r * Math.cos(joystick_V.ang) + joystickcenter.x;
			pos.y = joystickcenter.y - Round_r * Math.sin(joystick_V.ang);
			Pos_Vector(pos);
			joy_ctx.clearRect(0,0,windowWidth,windowHeight);
			drawjoystick(pos.x,pos.y);
		}
	}
})

joystick_canvas.addEventListener("mouseup", function(e){
	var pos = getMousePos(joystick_canvas, e);
	Pos_Vector(pos);
	if(mouse_click == 1){
		mouse_click = 0;
		joystick_V.x = 0;
		joystick_V.y = 0;
		joy_ctx.clearRect(0,0,joystick_canvas.width,joystick_canvas.height);
		drawjoystick(joystickcenter.x,joystickcenter.y);
	}
})

joystick_canvas.addEventListener("mouseleave", function(e){
	var pos = getMousePos(joystick_canvas, e);
	if(mouse_click == 1){
		pos.x = joystickcenter.x;
		pos.y = joystickcenter.y;
		Pos_Vector(pos);
		mouse_click = 0;
		joy_ctx.clearRect(0,0,joystick_canvas.width,joystick_canvas.height);
		drawjoystick(joystickcenter.x,joystickcenter.y);
	}
})

 function blockMenu(Evt){
                // window.event 是IE才有的物件
            if(window.event){
                Evt = window.event;
                Evt.returnValue = false;//取消IE預設事件
            }else
                Evt.preventDefault();//取消DOM預設事件
            }
            document.oncontextmenu = blockMenu;

