function MonitorSwitch(checked) {
    document.getElementById('canvasMap').style.zIndex = "0";
    var video = document.getElementById("canvasMap");
    let ground_reverse = document.getElementById("GroundButton").checked;
   
    if (checked == true) {
        video.src = "img/black.png";
        if(ground_reverse==true){
            document.getElementById('canvasMap').style.webkitTransform = "rotate(180deg)";
        }
        setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/mcl/image";},100);
        //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/mcl/image";
        console.log("localization map");
    } else {
        document.getElementById('canvasMap').style.webkitTransform = "rotate(0deg)";
        video.src = "img/LcGround.png";
        if (ground_reverse == false) {
            video.src = "img/LcGround.png";
        }else{
            video.src = "img/LcGround2.png";
        }
        console.log("ground map");
    }
}
function WhielineSwitch(checked) {
    var video = document.getElementById("canvasMap");
    let ground_reverse = document.getElementById("GroundButton").checked;
   
    if (checked == true) {
        video.src = "img/black.png";
        document.getElementById('canvasMap').style.webkitTransform = "rotate(0deg)";
        document.getElementById('canvasMap').style.zIndex = "999";
        setTimeout(function(){video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/white";},100);
        //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/mcl/image";
        console.log("whieline map");
    } else {
        document.getElementById('canvasMap').style.webkitTransform = "rotate(0deg)";
        document.getElementById('canvasMap').style.zIndex = "0";
        video.src = "img/LcGround.png";
        if (ground_reverse == false) {
            video.src = "img/LcGround.png";
        }else{
            video.src = "img/LcGround2.png";
        }
        console.log("ground map");
    }
}
function GroundSwitch(checked) {
    document.getElementById('canvasMap').style.zIndex = "0";
    mouse_clicked = false;
    reset_bool=false;
    let ground_reverse = document.getElementById("GroundButton").checked;
    let pi=3.14159;
    let canvas = document.getElementById('reset_map');
    let ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height);
    
    //console.log('111111');
//==================
    let video = document.getElementById("canvasMap");
    let map = document.getElementById("LocalizationButton").checked;
    let whiteline = document.getElementById("WhitelineButton").checked;
    if(map == true){
        if (checked == true) {
            console.log("reverse_checked");
            document.getElementById('canvasMap').style.webkitTransform = "rotate(180deg)";
        } else {
            document.getElementById('canvasMap').style.webkitTransform = "rotate(0deg)";
            console.log("ground map");
        }
    }
    else{
        if(whiteline==false){
          if (checked == false) {
              video.src = "img/LcGround.png";
              //console.log("11111111");
          }else{
              video.src = "img/LcGround2.png";
          }
        }
    }


    //let canva;
	//let ctx;

    let map_width = 1000;
    let map_height = 723;

    //console.log(map_width,map_height);
    //console.log(map_width-points[0][0],map_height-points[0][1]);
    ClearAllRectangle();
    //====================================
    canvas = document.getElementById('path_map');
    ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height); 
    let center_x = canvas.width/2;
    let center_y = canvas.height/2;
    if(checked){
        for(let i=0; i<Path.length; i+=2){
            console.log(Path[i],Path[i+1]);

            ctx.beginPath();
            ctx.font = "50px Times New Roman";
            ctx.fillStyle="white";
            ctx.fillText(i/2+1,center_x-Path[i]*1.3-10,center_y+Path[i+1]*1.3+15);
            ctx.closePath();
        }

    }else{
        for(let i=0; i<Path.length; i+=2){
            console.log(Path[i],Path[i+1]);

            ctx.beginPath();
            ctx.font = "50px Times New Roman";
            ctx.fillStyle="white";
            ctx.fillText(i/2+1,center_x+Path[i]*1.3-10,center_y-Path[i+1]*1.3+15);
            ctx.closePath();
        }
    }
    //===================================
}

var ResetMap = document.getElementById("reset_map");
var mouse_clicked = false;
var angle_offset=0;
var mouse_angle = 0;
ResetMap.addEventListener("mousedown", function(e) {
    //console.log("reset_map");
    if(e.button == 0){
        mouse_clicked = true;
        reset_bool = true;
        //console.log("0");
    }
    if(e.button == 2){
        mouse_clicked = false;
        reset_bool = false;
        //console.log("2");
    }
    let ground_reverse = document.getElementById("GroundButton").checked;
    let pi=3.14159;
    let canvas = document.getElementById('reset_map');
    let ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height);
    //=========================================
    if(mouse_clicked == true){
        let center_x = canvas.width/2;
        let center_y = canvas.height/2;
        let mouse_x = event.offsetX;
        let mouse_y = event.offsetY;
        let x_=(mouse_x-center_x)/1.33;
        let y_=(mouse_y-center_y)/1.33;
        
        let line_x = mouse_x+40 * Math.cos(mouse_angle);
        let line_y = mouse_y+40 * Math.sin(mouse_angle);
        //console.log(x_, y_);
        ctx.beginPath();
        ctx.lineWidth = 2;
        ctx.strokeStyle = '#FF3EFF';
        ctx.arc(mouse_x, mouse_y, 30, 0, 2*Math.PI);
        ctx.moveTo(mouse_x, mouse_y);
        ctx.lineTo(line_x, line_y);
        ctx.stroke();
        if(!ground_reverse){
            reset_x=x_;
            reset_y=y_;
            reset_w=-mouse_angle/pi*180;
        }else{
            reset_x=-x_;
            reset_y=-y_;
            reset_w=-((mouse_angle/pi*180)-180);
        }
        console.log(parseInt(reset_x), parseInt(reset_y), parseInt(reset_w));
        ResetMap.addEventListener("mousemove", function(event) {
            if(mouse_clicked == true){
                let canvas = document.getElementById('reset_map');
                let ctx=canvas.getContext("2d");
                ctx.clearRect(0,0,canvas.width,canvas.height);
                ctx.beginPath();
                let move_x = event.offsetX;
                mouse_angle = angle_offset+(mouse_x-move_x)/180*pi;
                line_x = mouse_x+40 * Math.cos(mouse_angle);
                line_y = mouse_y+40 * Math.sin(mouse_angle);
                ctx.arc(mouse_x, mouse_y, 30, 0, 2*Math.PI);
                ctx.moveTo(mouse_x, mouse_y);
                ctx.lineTo(line_x, line_y);
                ctx.stroke();
                if(!ground_reverse){
                    reset_x=x_;
                    reset_y=y_;
                    reset_w=-mouse_angle/pi*180;
                }else{
                    reset_x=-x_;
                    reset_y=-y_;
                    reset_w=-((mouse_angle/pi*180)-180);
                }
            }
        });
        
    }
});
ResetMap.addEventListener("mouseup", function(e) {
    mouse_clicked = false;
    //console.log("mouseup");
    angle_offset=mouse_angle;
});
ResetMap.addEventListener("mouseleave", function(e) {
    mouse_clicked = false;
    //console.log("mouseleave");
});
