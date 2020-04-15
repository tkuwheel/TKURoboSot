var dynaRecClient = new ROSLIB.Service({
    ros : ros3,
    name : 'robot1/blocker/set_parameters',
    serviceType : 'dynamic_reconfigure/Reconfigure'
});
var dynaRecClient2 = new ROSLIB.Service({
    ros : ros3,
    name : 'robot2/core/set_parameters',
    serviceType : 'dynamic_reconfigure/Reconfigure'
});
var dynaRecClient3 = new ROSLIB.Service({
    ros : ros3,
    name : 'robot3/core/set_parameters',
    serviceType : 'dynamic_reconfigure/Reconfigure'
});
var dynaRecClient4 = new ROSLIB.Service({
    ros : ros3,
    name : 'robot4/blocker/set_parameters',
    serviceType : 'dynamic_reconfigure/Reconfigure'
});
var dynaRecClient5 = new ROSLIB.Service({
    ros : ros3,
    name : 'robot5/core/set_parameters',
    serviceType : 'dynamic_reconfigure/Reconfigure'
});
var dynaRecClient6 = new ROSLIB.Service({
    ros : ros3,
    name : 'robot6/core/set_parameters',
    serviceType : 'dynamic_reconfigure/Reconfigure'
});
function processFormData() {
    var request = new ROSLIB.ServiceRequest({
        config: {
            bools: [
                {name: 'game_start', value: document.getElementById("game_start").checked},
                {name: 'chase_straight', value: document.getElementById("chase_straight").checked},
                {name: 'Accelerate', value: document.getElementById("Accelerate").checked},
                {name: 'ball_pwm', value: document.getElementById("ball_pwm").checked},
                {name: 'shooting_start', value: document.getElementById("shooting_start").checked},
            ],
            ints: [
                {name: 'ballhandle_dis', value: parseInt(document.getElementById("ballhandle_disInput").value)},
                {name: 'ballhandle_ang', value: parseInt(document.getElementById("ballhandle_angInput").value)},
            ],
            strs: [
                {name: 'role', value: document.getElementById("role").value},
                {name: 'game_state', value: document.getElementById("game_state").value},
                {name: 'run_point', value: document.getElementById("run_point").value},
                {name: 'our_side', value: document.getElementById("our_side").value},
                {name: 'attack_mode', value: document.getElementById("attack_mode").value},
                {name: 'strategy_mode', value: document.getElementById("strategy_mode").value},
            ],
            doubles: [
                {name: 'run_x', value: parseFloat(document.getElementById("run_xInput").value)},
                {name: 'run_y', value: parseFloat(document.getElementById("run_yInput").value)},
                {name: 'run_yaw', value: parseFloat(document.getElementById("run_yawInput").value)},
                {name: 'orb_attack_ang', value: parseFloat(document.getElementById("orb_attack_angInput").value)},
                {name: 'atk_shoot_ang', value: parseFloat(document.getElementById("atk_shoot_angInput").value)},
                {name: 'atk_shoot_dis', value: parseFloat(document.getElementById("atk_shoot_disInput").value)},
                {name: 'minimum_w', value: parseFloat(document.getElementById("minimum_wInput").value)},
                {name: 'maximum_w', value: parseFloat(document.getElementById("maximum_wInput").value)},
                {name: 'minimum_v', value: parseFloat(document.getElementById("minimum_vInput").value)},
                {name: 'maximum_v', value: parseFloat(document.getElementById("maximum_vInput").value)},
            ],
            groups: [
                // {name: '', state: false, id: 0, parent: 0}
            ]
        }
    });
    if(ChooseRobot==1){
        dynaRecClient.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });
    }else if(ChooseRobot==2){
        dynaRecClient2.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });
    }else if(ChooseRobot==3){
        dynaRecClient3.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });
    }
    if(document.getElementById("our_side").value=="Yellow" && Robot_Ctl!=null){
            $('#YellowButton').prop('checked',true);
            $('#YellowButton').change();
            attack_way();
        }
    if(document.getElementById("our_side").value=="Blue" && Robot_Ctl!=null){
        $('#BlueButton').prop('checked',true);
        $('#BlueButton').change();
        attack_way();
    }   
}
function change_side(){
    if(document.getElementById("YellowButton").checked==true && Robot_Ctl!=null){
        document.getElementById("our_side").value="Yellow";
    }
    if(document.getElementById("BlueButton").checked==true && Robot_Ctl!=null){
        document.getElementById("our_side").value="Blue";
    }
    processFormData();
}
function start_state(){
    if(document.getElementById("StopInput").checked == true && Robot_Ctl!=null){
        document.getElementById("game_start").checked = false;
        StrategyStop();
    }
    if(document.getElementById("StartInput").checked == true && Robot_Ctl!=null){
        document.getElementById("game_start").checked = true;
    }
    gameStart();
}
function gameStart(){
    var start = false;
    
    var request = new ROSLIB.ServiceRequest({
        config: {
            bools: [
                {name: 'game_start', value: document.getElementById("game_start").checked},
            ]
        }
    });

    dynaRecClient.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });

    dynaRecClient2.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });
    dynaRecClient3.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });
    dynaRecClient4.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });
    dynaRecClient5.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });
    dynaRecClient6.callService(request, function(result) {
            //console.log('Result for service call on '
            //    + dynaRecClient.name
            //    + ': '
            //    + JSON.stringify(result, null, 2));
        });

    vec3 = new ROSLIB.Message({
        x: 0,
        y: 0,
        z: 0
    });
    if(document.getElementById("game_start").checked==false){
        all_stop();
    }
}
function all_stop(){
    document.getElementById("game_start").checked = false;
    var request = new ROSLIB.ServiceRequest({
        config: {
            bools: [
                {name: 'game_start', value: false},
            ]
        }
    });
    dynaRecClient.callService(request, function(result) {
        //console.log('Result for service call on '
        //    + dynaRecClient.name
        //    + ': '
        //    + JSON.stringify(result, null, 2));
    });
    dynaRecClient2.callService(request, function(result) {
        //console.log('Result for service call on '
        //    + dynaRecClient.name
        //    + ': '
        //    + JSON.stringify(result, null, 2));
    });
    dynaRecClient3.callService(request, function(result) {
        //console.log('Result for service call on '
        //    + dynaRecClient.name
        //    + ': '
        //    + JSON.stringify(result, null, 2));
    });
    dynaRecClient4.callService(request, function(result) {
        //console.log('Result for service call on '
        //    + dynaRecClient.name
        //    + ': '
        //    + JSON.stringify(result, null, 2));
    });
    dynaRecClient5.callService(request, function(result) {
        //console.log('Result for service call on '
        //    + dynaRecClient.name
        //    + ': '
        //    + JSON.stringify(result, null, 2));
    });
    dynaRecClient6.callService(request, function(result) {
        //console.log('Result for service call on '
        //    + dynaRecClient.name
        //    + ': '
        //    + JSON.stringify(result, null, 2));
    });
}
function point_input(){
    reset_bool=false;
    let canvas = document.getElementById('reset_map');
    let ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height); 
    let angle = reset_w;
    if(angle>180)angle=360-angle;
    if(angle<-180)angle=angle+360;
    document.getElementById("run_xSlider").value = parseInt(reset_x);
    document.getElementById("run_xInput").value = parseInt(reset_x);

    document.getElementById("run_ySlider").value = parseInt(-reset_y);
    document.getElementById("run_yInput").value = parseInt(-reset_y);

    document.getElementById("run_yawSlider").value = parseInt(angle);
    document.getElementById("run_yawInput").value = parseInt(angle);

    processFormData();
}
function change_maxw(value){
    var w = 10
    if(value<80){
        //公式：Y = ( ( X - X1 )( Y2 - Y1) / ( X2 - X1) ) + Y1
        //v10-80 w10-40
        var w = (value-10)*(40-10)/(80-10)+10
        if(w>40)w=40
        if(w<15)w=15
        document.getElementById("maximum_wSlider").value = w;
        document.getElementById("maximum_wInput").value = w;
    }
}
