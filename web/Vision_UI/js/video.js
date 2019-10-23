
//===========================================================================
//new 

Element.prototype.leftTopScreen = function() {
    var x = this.offsetLeft;
    var y = this.offsetTop;
    var element = this.offsetParent;

    while (element !== null) {
        x = parseInt(x) + parseInt(element.offsetLeft);
        y = parseInt(y) + parseInt(element.offsetTop);

        element = element.offsetParent;
    }
    return new Array(x, y);
}

function Angle_Adjustment(angle) {
    if (angle < 0) {
        angle = angle + 360;
    } else if (angle >= 360) {
        angle = angle - 360;
    }
    return angle;
}

function Strategy_Angle(angle) {
    var front = document.getElementsByName('CenterElement')[4].value;
    angle = angle-front;
    angle = Angle_Adjustment(angle);
    if (angle > 180) {
        angle = angle - 360;
    }
    return angle;
}
function Angle_Calculation(mouseX, mouseY) {
    var centerX = document.getElementsByName('CenterElement')[0].value;
    var centerY = document.getElementsByName('CenterElement')[1].value;
    var front = document.getElementsByName('CenterElement')[4].value;
    y = mouseY - centerY;
    x = mouseX - centerX;
    var angle = Math.floor(Math.atan2(-y, x) * 180 * 10 / Math.PI) / 10;

    angle = Strategy_Angle(angle);
    angle=Math.floor(angle*10)/10;
    document.getElementById("CameraAngle").innerText = angle;
}
video_canvas.addEventListener("mousedown", function(e) {
    var flip = document.getElementById("playerfunction");

    var xy = flip.leftTopScreen();
    var centerX = document.getElementsByName('CenterElement')[0].value;
    var centerY = document.getElementsByName('CenterElement')[1].value;
    var context = flip.getContext("2d");

    context.fillStyle = 'rgba(255, 255, 255, 0)';
    context.fillRect(0, 0, 659, 493);
    
    //flip.addEventListener("click", function(event) {
        var x = event.clientX;
        var y = event.clientY;

        document.getElementById("CameraX").innerText = Math.floor(x*0.966 - xy[0]*0.966 - centerX);
        document.getElementById("CameraY").innerText = Math.floor(centerY - y*0.966 + xy[1]*0.966) ;

        //document.getElementById("CameraX").innerText = x - xy[0] - centerX;
        //document.getElementById("CameraY").innerText = centerY - y + xy[1];

        Angle_Calculation((x - xy[0])*0.966, (y - xy[1])*0.966);
        topicROSPosition((x - xy[0])*0.966, (y - xy[1])*0.966);
        //console.log(x - xy[0], y - xy[1]);
    //});
});

function CheckCamera(check) {
    var video = document.getElementById("player");
    if (check)
        video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=camera/image";
    else
        video.src = "img/offline.png";

}
function MonitorCheck(value){
    var video = document.getElementById("player");
    if(value==1){
      video.src = "img/offline.png";
    }
}

var monitor;
function MonitorSwitch(value) {
    var video = document.getElementById("player");
    var check = document.getElementById("CameraSwitch").checked;
    //console.log("open");
    //console.log(value);
    if(value==8){
      	video.src = "img/offline.png";
        monitor=8;
    }
    //console.log(monitor);
    if(value==2){monitor=2;}
    if (value == 7){	
        if (check)
            video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=camera/image_monitor";
        else
            video.src = "img/offline.png";
    }
    else if(monitor!=8) {
	//console.log(monitor);
        if (check)
            video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=camera/image";
        else
            video.src = "img/offline.png";
    }
    if(value==false)video.src = "img/offline.png";
}
