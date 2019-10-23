function GetJoystickSpeed() {
    var speed = document.getElementById("SpeedInput").value;
    var Xspeed = parseFloat((joystick_V.x / Round_r) * speed);
    var Yspeed = parseFloat((joystick_V.y / Round_r) * speed);

    if (Xspeed >= 100) {
        Xspeed = 100;
    }
    if (Yspeed >= 100) {
        Yspeed = 100;
    }
    var vec3 = new ROSLIB.Message({
        x: Xspeed,
        y: Yspeed,
        z: 0
    });
    PublishTopicCmdVel(vec3);
}

function drawjoystick_base() {
    joy_ctx.beginPath();
    joy_ctx.lineWidth = 1;
    joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, Round_r, 0, 2 * Math.PI);
    joy_ctx.moveTo(joystick_canvas.width / 2, (joystick_canvas.height / 2) - Round_r);
    joy_ctx.lineTo(joystick_canvas.width / 2, (joystick_canvas.height / 2) + Round_r);
    joy_ctx.moveTo((joystick_canvas.width / 2) - Round_r, joystick_canvas.height / 2);
    joy_ctx.lineTo((joystick_canvas.width / 2) + Round_r, joystick_canvas.height / 2);
    joy_ctx.globalAlpha = 0.7;
    joy_ctx.stroke();

    GetJoystickSpeed();
}

function drawjoystick_base_rightkey() {
    joy_ctx.beginPath();
    joy_ctx.lineWidth = 1;
    joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, Round_r, 0, 2 * Math.PI);
    joy_ctx.moveTo(joystick_canvas.width / 2, (joystick_canvas.height / 2) - Round_r);
    joy_ctx.lineTo(joystick_canvas.width / 2, (joystick_canvas.height / 2) + Round_r);
    joy_ctx.moveTo((joystick_canvas.width / 2) - Round_r, joystick_canvas.height / 2);
    joy_ctx.lineTo((joystick_canvas.width / 2) + Round_r, joystick_canvas.height / 2);
    joy_ctx.globalAlpha = 0.7;
    joy_ctx.stroke();

}

function getMousePos(cv, e) {
    var rect = cv.getBoundingClientRect();
    return {
        x: e.clientX - rect.left, //相對於Canvas左上角的X座標
        y: e.clientY - rect.top, //相對於Canvas左上角的Y座標
        rectLeft: rect.left,
        rectTop: rect.top,
        clientX: e.clientX,
        clientY: e.clientY
    }
}

function Pos_Vector(_pos) {
    joystick_V.x = _pos.x - joystickcenter.x;
    joystick_V.y = joystickcenter.y - _pos.y;
    joystick_V.ang = Math.atan2(joystick_V.y, joystick_V.x);
}


function drawjoystick(x, y) {
    drawjoystick_base();
    joy_ctx.beginPath();
    joy_ctx.arc(x, y, 8, 0, 2 * Math.PI);
    joy_ctx.globalAlpha = 1;
    joy_ctx.lineWidth = 16;
    joy_ctx.stroke();
}

function joystick_main() {
    joystick_canvas.width = joystick_Area.offsetWidth;
    joystick_canvas.height = joystick_Area.offsetHeight;
    drawjoystick(joystickcenter.x, joystickcenter.y);
}

function checkArea() {
    var line = Math.sqrt(Math.pow(joystick_V.x, 2) + Math.pow(joystick_V.y, 2));
    if (line > 120) {
        return 0;
    } else {
        return 1;
    }
}

joystick_canvas.addEventListener("mouseenter", function(e) {
    var pos = getMousePos(joystick_canvas, e);
    Pos_Vector(pos);
})

joystick_canvas.addEventListener("mousedown", function(e) {
    var pos = getMousePos(joystick_canvas, e);
    Pos_Vector(pos);
    console.log(e.button);
    logButton = e.button;
    if (e.button == 0) {
        if (checkArea() == 1) {
            joy_ctx.clearRect(0, 0, windowWidth, windowHeight);
            drawjoystick(pos.x, pos.y);
            mouse_click = 1;
        }
    } else if (e.button == 1) {
        if (checkArea() == 1) {
            joy_ctx.clearRect(0, 0, windowWidth, windowHeight);
            joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, Round_r, 0, 2 * Math.PI);
            joy_ctx.globalAlpha = 1;
            joy_ctx.lineWidth = 16;
            joy_ctx.fillStyle = "black"; //填充颜色,默认是黑色
            joy_ctx.fill(); //画实心圆*/
            boom.src = 'img/boom.png';
            mouse_click = 1;

            PublishTopicShoot(parseInt(document.getElementById('ShootInput').value));
        }
    } else if (e.button == 2) {
        if (checkArea() == 1) {
            joy_ctx.clearRect(0, 0, windowWidth, windowHeight);
            drawjoystick_base_rightkey();
            var radius = Math.sqrt(Math.pow((joystick_canvas.width / 2) - pos.x, 2) + Math.pow((joystick_canvas.height / 2) - pos.y, 2));
            var c_length = Math.sqrt(Math.pow((joystick_canvas.width / 2) - pos.x, 2) + Math.pow((joystick_canvas.height / 2) - radius - pos.y, 2));
            var h = Math.sqrt(Math.pow(radius, 2) - Math.pow(c_length / 2, 2));
            var angle = Math.PI - 2 * Math.asin(h / radius);
            var area = Math.pow(radius, 2) * angle / 2;
            var rate = area / (Math.pow(Round_r, 2) * Math.PI / 2);
            var speed = document.getElementById("SpeedInput").value;
            var vec3 = new ROSLIB.Message({
                x: 0,
                y: 0,
                z: 0
            });
            if (radius >= Round_r)
                radius = Round_r;
            joy_ctx.beginPath();
            joy_ctx.moveTo(joystick_canvas.width / 2, joystick_canvas.height / 2);
            if (((joystick_canvas.width / 2) - pos.x) < 0) {
                joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, radius, 270 * Math.PI / 180, angle + 270 * Math.PI / 180, false);
                var vec3 = new ROSLIB.Message({
                    x: 0,
                    y: 0,
                    z: -parseFloat(1 * rate * speed)
                });
            } else {
                joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, radius, 270 * Math.PI / 180 - angle, 270 * Math.PI / 180, false);
                var vec3 = new ROSLIB.Message({
                    x: 0,
                    y: 0,
                    z: parseFloat(1 * rate * speed)
                });
            }
            joy_ctx.closePath();
            joy_ctx.fillStyle = '#FFCEBE';
            joy_ctx.fill();
            //console.log(vec3);
            PublishTopicCmdVel(vec3);
            mouse_click = 1;
        }
    }
})
mouse_click = 1;

joystick_canvas.addEventListener("mousemove", function(e) {
    var pos = getMousePos(joystick_canvas, e);
    Pos_Vector(pos);
    if (mouse_click == 1) {
        if (logButton == 0) {
            if (checkArea()) {
                joy_ctx.clearRect(0, 0, windowWidth, windowHeight);
                drawjoystick(pos.x, pos.y);
            } else {
                pos.x = Round_r * Math.cos(joystick_V.ang) + joystickcenter.x;
                pos.y = joystickcenter.y - Round_r * Math.sin(joystick_V.ang);
                Pos_Vector(pos);
                joy_ctx.clearRect(0, 0, windowWidth, windowHeight);
                drawjoystick(pos.x, pos.y);
            }
        } else if (logButton == 1) {
            if (checkArea() == 1) {
                joy_ctx.clearRect(0, 0, windowWidth, windowHeight);
                joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, Round_r, 0, 2 * Math.PI);
                joy_ctx.globalAlpha = 1;
                joy_ctx.lineWidth = 16;
                joy_ctx.fillStyle = "black"; //填充颜色,默认是黑色
                joy_ctx.fill(); //画实心圆*/
                boom.src = 'img/boom.png';
            }
        } else if (logButton == 2) {
            if (checkArea() == 1) {
                joy_ctx.clearRect(0, 0, windowWidth, windowHeight);
                drawjoystick_base_rightkey();
                var radius = Math.sqrt(Math.pow((joystick_canvas.width / 2) - pos.x, 2) + Math.pow((joystick_canvas.height / 2) - pos.y, 2));
                var c_length = Math.sqrt(Math.pow((joystick_canvas.width / 2) - pos.x, 2) + Math.pow((joystick_canvas.height / 2) - radius - pos.y, 2));
                var h = Math.sqrt(Math.pow(radius, 2) - Math.pow(c_length / 2, 2));
                var angle = Math.PI - 2 * Math.asin(h / radius);
                var area = Math.pow(radius, 2) * angle / 2;
                var rate = area / (Math.pow(Round_r, 2) * Math.PI / 2);
                var speed = document.getElementById("SpeedInput").value;
                var vec3 = 0;
                if (radius >= Round_r)
                    radius = Round_r;

                joy_ctx.beginPath();
                joy_ctx.moveTo(joystick_canvas.width / 2, joystick_canvas.height / 2);
                if (((joystick_canvas.width / 2) - pos.x) < 0) {
                    joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, radius, 270 * Math.PI / 180, angle + 270 * Math.PI / 180, false);
                    var vec3 = new ROSLIB.Message({
                        x: 0,
                        y: 0,
                        z: -parseFloat(1 * rate * speed)
                    });
                } else {
                    joy_ctx.arc(joystick_canvas.width / 2, joystick_canvas.height / 2, radius, 270 * Math.PI / 180 - angle, 270 * Math.PI / 180, false);
                    var vec3 = new ROSLIB.Message({
                        x: 0,
                        y: 0,
                        z: parseFloat(1 * rate * speed)
                    });
                }
                joy_ctx.closePath();
                joy_ctx.fillStyle = '#FFCEBE';
                joy_ctx.fill();
                console.log(vec3);
                PublishTopicCmdVel(vec3);
            }
        }
    }

})

joystick_canvas.addEventListener("mouseup", function(e) {
    var pos = getMousePos(joystick_canvas, e);
    Pos_Vector(pos);
    if (mouse_click == 1) {
        mouse_click = 0;
        joystick_V.x = 0;
        joystick_V.y = 0;
        joy_ctx.clearRect(0, 0, joystick_canvas.width, joystick_canvas.height);
        drawjoystick(joystickcenter.x, joystickcenter.y);
        //setTimeout(StandBy, 0);
        //StandBy();
    }
})

joystick_canvas.addEventListener("mouseleave", function(e) {
    var pos = getMousePos(joystick_canvas, e);
    Pos_Vector(pos);
    if (mouse_click == 1) {
        mouse_click = 0;
        joystick_V.x = 0;
        joystick_V.y = 0;
        joy_ctx.clearRect(0, 0, joystick_canvas.width, joystick_canvas.height);
        drawjoystick(joystickcenter.x, joystickcenter.y);
    }
})