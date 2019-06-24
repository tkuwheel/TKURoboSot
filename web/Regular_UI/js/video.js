var monitor;

function MonitorSwitch(checked) {
    
    let video = document.getElementById("MapCanvas");
    //let check = document.getElementById("CameraSwitch").checked;

    if (checked == true) {
        let canvas = document.getElementById('reset_map');
        let ctx=canvas.getContext("2d");
        ctx.clearRect(0,0,canvas.width,canvas.height);
        canvas = document.getElementById('attack_map');
        ctx=canvas.getContext("2d");
        ctx.clearRect(0,0,canvas.width,canvas.height);
        canvas = document.getElementById("robot_map");
        ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        canvas = document.getElementById("caution_map");
        ctx = canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ViewButton(64);
        video.src = "img/black.png";
        if (ChooseRobot == 1) {
            setTimeout(function(){video.src = 'http://' + document.getElementById('RobotIP').value + ':8080/stream?topic=/camera/image_monitor'},100);
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/image_monitor";
            //console.log(11);
        }
        else if (ChooseRobot == 2) {
            setTimeout(function(){video.src = 'http://' + document.getElementById('RobotIP2').value + ':8080/stream?topic=/camera/image_monitor'},100);
            //video.src = "http://" + document.getElementById("RobotIP2").value + ":8080/stream?topic=/camera/image_monitor";
            //console.log(22);
        }
        else if (ChooseRobot == 3) {
            setTimeout(function(){video.src = 'http://' + document.getElementById('RobotIP3').value + ':8080/stream?topic=/camera/image_monitor'},100);
            //video.src = "http://" + document.getElementById("RobotIP3").value + ":8080/stream?topic=/camera/image_monitor";
            //console.log(33);
        }
        else{
            video.src = "img/black.png";
            let center_x = video.width / 2;
            let center_y = video.height / 2;
            ctx.save();
            ctx.font="25px Georgia";

            ctx.translate(center_x-75, center_y);
            ctx.fillStyle  = 'white';
            ctx.fillText("未選擇機器人", 0, 0);
            
            ctx.restore();
            // console.log(00);
        }
    } else {
        attack_way();
        
        //======================================
        if(reset_bool == true){
            let canvas = document.getElementById('reset_map');
            let ctx=canvas.getContext("2d");
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            ctx.beginPath();
            ctx.lineWidth = 2;
            ctx.strokeStyle = '#FF3EFF';
            ctx.arc(mouse_x, mouse_y, 30, 0, 2*Math.PI);
            ctx.moveTo(mouse_x, mouse_y);
            ctx.lineTo(line_x, line_y);
            ctx.stroke();
            
        }
        //======================================
        ViewButton(1);
        video.src = "img/ground.png";
        let ground_reverse = document.getElementById("GroundButton").checked;
        if (ground_reverse == true) {
            console.log("reverse_checked");
            document.getElementById('MapCanvas').style.webkitTransform = "rotate(180deg)";
            document.getElementById('robot_map').style.webkitTransform = "rotate(180deg)";
            document.getElementById('reset_map').style.webkitTransform = "rotate(180deg)";
        } else {
            document.getElementById('MapCanvas').style.webkitTransform = "rotate(0deg)";
            document.getElementById('robot_map').style.webkitTransform = "rotate(0deg)";
            document.getElementById('reset_map').style.webkitTransform = "rotate(0deg)";
            console.log("ground map");
        }
    }
}

function Mclmap(checked) {
    attack_way();
    var video = document.getElementById("MapCanvas");

    let canvas = document.getElementById("robot_map");
    let ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    canvas = document.getElementById("caution_map");
    ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    if(reset_bool == true){
        let canvas = document.getElementById('reset_map');
        let ctx=canvas.getContext("2d");
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.beginPath();
        ctx.lineWidth = 2;
        ctx.strokeStyle = '#FF3EFF';
        ctx.arc(mouse_x, mouse_y, 30, 0, 2*Math.PI);
        ctx.moveTo(mouse_x, mouse_y);
        ctx.lineTo(line_x, line_y);
        ctx.stroke();
        
    }
    if (checked == true) {
        video.src = "img/black.png";
        if (ChooseRobot == 1) {
            setTimeout(function(){video.src = 'http://' + document.getElementById('RobotIP').value + ':8080/stream?topic=/camera/image_monitor'},100);
            //video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/mcl/image";
            //console.log(11);
        }
        else if (ChooseRobot == 2) {
            //console.log("white")
            
            setTimeout(function(){video.src = 'http://' + document.getElementById('RobotIP2').value + ':8080/stream?topic=/mcl/image'},100);
           // video.src = "http://" + document.getElementById("RobotIP2").value + ":8080/stream?topic=/mcl/image";
            
            //console.log(22);
        }
        else if (ChooseRobot == 3) {
            setTimeout(function(){video.src = 'http://' + document.getElementById('RobotIP3').value + ':8080/stream?topic=/mcl/image'},100);
            //video.src = "http://" + document.getElementById("RobotIP3").value + ":8080/stream?topic=/mcl/image";
            //console.log(33);
        }
        else{
            video.src = "img/black.png";
            let center_x = video.width / 2;
            let center_y = video.height / 2;
            ctx.save();
            ctx.font="25px Georgia";
  
            ctx.translate(center_x-75, center_y);
            ctx.fillStyle  = 'white';
            ctx.fillText ("未選擇機器人", 0, 0);
            
            ctx.restore();
            // console.log(00);
        }

        
    } else {
        ViewButton(1);
        video.src = "img/ground.png";

        
    }
}
function attack_way(){
  var canvas = document.getElementById('attack_map'),
      context = canvas.getContext('2d'),
      image = new Image();
  let map_check = document.getElementById("CameraSwitch").checked;
  let blueteam = document.getElementById("BlueButton").checked;
  let yellowteam = document.getElementById("YellowButton").checked;
  let ground_reverse = document.getElementById("GroundButton").checked;
  if(map_check==false){
      if(ground_reverse==false){
        if(blueteam==true){
          image.src = 'img/left.png';
          image.onload = function() {
            var w = this.width,
                h = this.height;
              
            canvas.width = w;
            canvas.height = h;
            context.drawImage(this, 0, 0, w, h);
          }
        }
        if(yellowteam==true){
          image.src = 'img/right.png';
          image.onload = function() {
            var w = this.width,
                h = this.height;
              
            canvas.width = w;
            canvas.height = h;

            //context.rotate(Math.PI);
            context.drawImage(this, 0, 0, w, h);
           
          }
        }
      }else{
        if(blueteam==true){
          image.src = 'img/right.png';
          image.onload = function() {
            var w = this.width,
                h = this.height;
              
            canvas.width = w;
            canvas.height = h;
            context.drawImage(this, 0, 0, w, h);
          }
        }
        if(yellowteam==true){
          image.src = 'img/left.png';
          image.onload = function() {
            var w = this.width,
                h = this.height;
              
            canvas.width = w;
            canvas.height = h;

            //context.rotate(Math.PI);
            context.drawImage(this, 0, 0, w, h);
           
          }
        }
      }
  }
}
function GroundSwitch(checked) {
    attack_way();
    mouse_clicked = false;
    reset_bool=false;
    let ground_reverse = document.getElementById("GroundButton").checked;
    let pi=3.14159;
    let canvas = document.getElementById('reset_map');
    let ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height);

    //console.log('111111');
//=================
    let mclmap_click = document.getElementById("MCLmap").checked;
    canvas = document.getElementById("robot_map");
    ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    canvas = document.getElementById("caution_map");
    ctx = canvas.getContext("2d");
    ctx.clearRect(0, 0, canvas.width, canvas.height);
//==================
    let video = document.getElementById("MapCanvas");
    let map = document.getElementById("CameraSwitch").checked;
    if(map == false||ChooseRobot == null){
        if (checked == true) {
            console.log("reverse_checked");
            document.getElementById('MapCanvas').style.webkitTransform = "rotate(180deg)";
            document.getElementById('robot_map').style.webkitTransform = "rotate(180deg)";
            document.getElementById('reset_map').style.webkitTransform = "rotate(180deg)";
        } else {
            document.getElementById('MapCanvas').style.webkitTransform = "rotate(0deg)";
            document.getElementById('robot_map').style.webkitTransform = "rotate(0deg)";
            document.getElementById('reset_map').style.webkitTransform = "rotate(0deg)";
            console.log("ground map");
        }
    }
    else{
        if (checked == false) {
            //document.getElementById('MapCanvas').style.webkitTransform = "rotate(0deg)";
            //video.src = "img/Ground.png";
        }else{
            //document.getElementById('MapCanvas').style.webkitTransform = "rotate(180deg)";
            //video.src = "img/LcGround2.png";
        }
    }
    if(mclmap_click==true||map==true){

        if (ChooseRobot == null){
            let center_x = canvas.width / 2;
            let center_y = canvas.height / 2;
            ctx.save();
            ctx.font="25px Georgia";
            
            ctx.translate(center_x-75, center_y);
            ctx.fillStyle  = 'white';
            ctx.fillText("未選擇機器人", 0, 0);
            
            ctx.restore();
            // console.log(00);
        }
    }
}

    //====================================
    //let canva;
	//let ctx;

    //let map_width = 940;
    //let map_height = 666;

    var ResetMap = document.getElementById("reset_map");
    var mouse_clicked = false;
    var angle_offset=0;
    var mouse_angle = 0;
    ResetMap.addEventListener("mousedown", function(e) {
        //console.log("reset_map");
        if (ChooseRobot != null){
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
          let camera_on = document.getElementById("CameraSwitch").checked;
          let pi=3.14159;

          let canvas = document.getElementById("caution_map");
          let ctx = canvas.getContext("2d");
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          canvas = document.getElementById('reset_map');
          ctx=canvas.getContext("2d");
          ctx.clearRect(0,0,canvas.width,canvas.height);
          //=========================================
          if(mouse_clicked == true&&camera_on==false){
              let center_x = canvas.width/2;
              let center_y = canvas.height/2;
              mouse_x = event.offsetX;
              mouse_y = event.offsetY;
              let x_=(mouse_x-center_x)/1.33;
              let y_=(mouse_y-center_y)/1.33;
              
              line_x = mouse_x+40 * Math.cos(mouse_angle);
              line_y = mouse_y+40 * Math.sin(mouse_angle);
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
                  camera_on = document.getElementById("CameraSwitch").checked;
                  if(mouse_clicked == true&&camera_on==false){
                      //console.log("camera_on==false");
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
        }else{
          let ground_reverse = document.getElementById("GroundButton").checked;
          canvas = document.getElementById("caution_map");
          ctx = canvas.getContext("2d");
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          //ctx.save();
          ctx.font="25px Georgia";
          let center_x = canvas.width / 2;
          let center_y = canvas.height / 2;
          
          
          if(ground_reverse==false){
            //console.log(event.offsetY);
            //ctx.translate(event.offsetX-75, event.offsetY);
            ctx.fillStyle  = 'white';
            ctx.fillText("未選擇機器人",event.offsetX-75,event.offsetY); 
          }else{
            //console.log(event.offsetY);
            //ctx.translate(canvas.width-event.offsetX-75, canvas.height-event.offsetY);
            ctx.fillStyle  = 'white';
            ctx.fillText("未選擇機器人",canvas.width-event.offsetX-75,canvas.height-event.offsetY); 
          }
          
          //ctx.fillText("未選擇機器人", 0, 0);
          
          //ctx.restore();
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

