var light;
var canvas = document.getElementById("Light");
var context = canvas.getContext("2d");
context.font = '30pt Calibri';
context.arc(20, 20, 20, 0, Math.PI * 2, false);
//////////////////////////////////////////////////////
    var RegionTimes = -1;
    var Order = [0,0,0,0,0];
    var Checkorder = [0,0,0,0,0];
///////////////////////////////////Joystick
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
//========================================================
function RegionReset() {
    var obj = document.getElementsByName("LocalElement1");
    var canva;
    var ctx;
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == true) {
            canva = document.getElementById("Rect"+document.getElementsByName("LocalElement1")[i].value);
            ctx = canva.getContext("2d");
            ctx.clearRect(0,0,60,60);
            document.getElementsByName("LocalElement1")[i].checked = false;     
        }
    }
    obj = document.getElementsByName("LocalElement2");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == true) {
            canva = document.getElementById("Rect"+document.getElementsByName("LocalElement2")[i].value);
            ctx = canva.getContext("2d");
            ctx.clearRect(0,0,60,60);
            document.getElementsByName("LocalElement2")[i].checked = false;     

        }
    }
    obj = document.getElementsByName("LocalElement3");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == true) {
            canva = document.getElementById("Rect"+document.getElementsByName("LocalElement3")[i].value);
            ctx = canva.getContext("2d");
            ctx.clearRect(0,0,60,60);
            document.getElementsByName("LocalElement3")[i].checked = false;     

        }
    }
    obj = document.getElementsByName("LocalElement4");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == true) {
            canva = document.getElementById("Rect"+document.getElementsByName("LocalElement4")[i].value);
            ctx = canva.getContext("2d");
            ctx.clearRect(0,0,60,60);
            document.getElementsByName("LocalElement4")[i].checked = false;     

        }
    }
    obj = document.getElementsByName("LocalElement5");
    for (var i=0; i<obj.length; i++) {
        if (obj[i].checked == true) {
            canva = document.getElementById("Rect"+document.getElementsByName("LocalElement5")[i].value);
            ctx = canva.getContext("2d");
            ctx.clearRect(0,0,60,60);
            document.getElementsByName("LocalElement5")[i].checked = false;     

        }
    }
    RegionTimes = -1;
    for (i = 0; i < 5; i++) {
        Order[i] = 0;
        Checkorder[i] = 0;
    }
}
function RegionOrder(region, flag) {
    var i;
    var checked = 0;
    if (RegionTimes == -1) {
        Order[++RegionTimes] = region;
        Checkorder[RegionTimes] = flag;
    }
    for (i = 0; i < 5; i++) {
        if (Checkorder[i] == flag) {
            Order[i] = region;
            checked = 1;
        }
    }
    if (checked == 0) {
        Checkorder[++RegionTimes] = flag;
        Order[RegionTimes] = region;
    }
    console.log(Checkorder, Order);
}
function RegionLocation() {
    var i, j = -1;
    var Box = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
    for (i = 0; i < 5; i++) {
        switch (Order[i]) {
            case 1:

                Box[++j] = parseFloat(-280);
                Box[++j] = parseFloat(180);
                break;
            case 2:
                Box[++j] = parseFloat(-75);
                Box[++j] = parseFloat(180);
                break;
            case 3:
                Box[++j] = parseFloat(75);
                Box[++j] = parseFloat(180);
                break;
            case 4:
                Box[++j] = parseFloat(280);
                Box[++j] = parseFloat(180);
                break;
            case 5:
                Box[++j] = parseFloat(-225);
                Box[++j] = parseFloat(105);
                break;
            case 6:
                Box[++j] = parseFloat(-122.5);
                Box[++j] = parseFloat(95);
                break;
            case 7:
                Box[++j] = parseFloat(0);
                Box[++j] = parseFloat(90);
                break;
            case 8:
                Box[++j] = parseFloat(122.5);
                Box[++j] = parseFloat(95);
                break;
            case 9:
                Box[++j] = parseFloat(225);
                Box[++j] = parseFloat(105);
                break;
            case 10:
                Box[++j] = parseFloat(-260);
                Box[++j] = parseFloat(0);
                break;
            case 11:
                Box[++j] = parseFloat(-165);
                Box[++j] = parseFloat(0);
                break;
            case 12:
                Box[++j] = parseFloat(165);
                Box[++j] = parseFloat(0);
                break;
            case 13:
                Box[++j] = parseFloat(260);
                Box[++j] = parseFloat(0);
                break;
            case 14:
                Box[++j] = parseFloat(-225);
                Box[++j] = parseFloat(-105);
                break;
            case 15:
                Box[++j] = parseFloat(-122.5);
                Box[++j] = parseFloat(-95);
                break;
            case 16:
                Box[++j] = parseFloat(0);
                Box[++j] = parseFloat(-90);
                break;
            case 17:
                Box[++j] = parseFloat(122.5);
                Box[++j] = parseFloat(-95);
                break;
            case 18:
                Box[++j] = parseFloat(225);
                Box[++j] = parseFloat(-105);
                break;
            case 19:
                Box[++j] = parseFloat(-280);
                Box[++j] = parseFloat(-180);
                break;
            case 20:
                Box[++j] = parseFloat(-75);
                Box[++j] = parseFloat(-180);
                break;
            case 21:
                Box[++j] = parseFloat(75);
                Box[++j] = parseFloat(-180);
                break;
            case 22:
                Box[++j] = parseFloat(280);
                Box[++j] = parseFloat(-180);
                break;
        }
    }
    for (i = 0; i < 10; i += 2) {
        var temp = 0;
        temp = Box[i];
        Box[i] = -Box[i + 1];
        Box[i + 1] = temp;
    }
    console.log(Order);
    console.log(Box);
    DrawOrderRectangle();
    TopicRegion(Box);
  }




// function cnvs_getCoordinates(e){
//     x=e.clientX;
//     y=e.clientY;
//     document.getElementById("xycoordinates ").innerHTML="Coordinates: ( " + x + ", " + y + ") ";
// }
// function cnvs_clearCoordinates(){
//     document.getElementById("xycoordinates ").innerHTML=" ";
// }
//===========================================================
function Optimization() {
    var CheckBox = [];
    var obj = document.getElementsByName("OptimizationElement");
    for (var i = 0; i < obj.length; i++) {
        if (obj[i].checked) {
            CheckBox.push(parseInt(1));
        } else {
            CheckBox.push(parseInt(0));
        }
    }
    TopicOptimization(CheckBox);
    console.log(CheckBox);
}
//===========================================================

//service
// -----------------

// var updateClient = new ROSLIB.Service({
//     ros: ros,
//     name: '/StrategyParam',
//     serviceType: 'param_convey/strategy_param'
// });

// var request = new ROSLIB.ServiceRequest({
//     receive: 1
// });

// function up() {
//     document.getElementById("Update").style.cursor = "wait";
//     context2.fillStyle = "yellow";
//     context2.fill();
//     updateClient.callService(request, function(res) {
//         if (res.update == 2) {
//             document.getElementById("Update").style.cursor = "default";
//             context2.fillStyle = "green";
//             context2.fill();
//         }
//     });

// }
///////////////////////joystickfunction///////////////////////
function ToInputValue(newValue, name, num) {
    document.getElementsByName(name)[num].value = newValue;
}

function ToSliderValue(newValue, name, num) {
    if (newValue > 100) {
        newValue = 100;
    }
    document.getElementsByName(name)[1].value = newValue;
    document.getElementsByName(name)[num].value = newValue;
}
