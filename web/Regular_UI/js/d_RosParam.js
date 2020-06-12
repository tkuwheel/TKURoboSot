var Robot_Ctl = null;
var role_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/role'
});
var gamestrat_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/game_start'
});

var chase_straight_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/chase_straight'
});

var Accelerate_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/Accelerate'
});

var ball_pwm_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/ball_pwm'
});

var shooting_start_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/shooting_start'
});

var game_state_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/game_state'
});

var run_point_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/run_point'
});

var run_x_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/run_x'
});

var run_y_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/run_y'
});

var run_yaw_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/run_yaw'
});

var our_side_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/our_side'
});

var attack_mode_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/attack_mode'
});

var strategy_mode_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/strategy_mode'
});

var orb_attack_ang_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/orb_attack_ang'
});

var atk_shoot_ang_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/atk_shoot_ang'
});

var atk_shoot_dis_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/atk_shoot_dis'
});

var minimum_w_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/minimum_w'
});

var maximum_w_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/maximum_w'
});

var minimum_v_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/minimum_v'
});

var maximum_v_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/maximum_v'
});

var ballhandle_dis_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/ballhandle_dis'
});

var ballhandle_ang_param = new ROSLIB.Param({
    ros: Robot_Ctl,
    name: 'core/ballhandle_ang'
});
function change_robot(value){
    if(value==1){
        Robot_Ctl = ros
    }else if(value==2){
        Robot_Ctl = ros2
    }else if(value==3){
        Robot_Ctl = ros3
    }
    rebuild_parameter();
    setTimeout(get_dynamic_parameter, 400);
}
function rebuild_parameter(){
    role_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/role'
    });
    gamestrat_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/game_start'
    });

    chase_straight_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/chase_straight'
    });

    Accelerate_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/Accelerate'
    });

    ball_pwm_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/ball_pwm'
    });

    shooting_start_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/shooting_start'
    });

    game_state_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/game_state'
    });

    run_point_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/run_point'
    });

    run_x_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/run_x'
    });

    run_y_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/run_y'
    });

    run_yaw_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/run_yaw'
    });

    our_side_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/our_side'
    });

    attack_mode_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/attack_mode'
    });

    strategy_mode_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/strategy_mode'
    });

    orb_attack_ang_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/orb_attack_ang'
    });

    atk_shoot_ang_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/atk_shoot_ang'
    });

    atk_shoot_dis_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/atk_shoot_dis'
    });

    minimum_w_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/minimum_w'
    });

    maximum_w_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/maximum_w'
    });

    minimum_v_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/minimum_v'
    });

    maximum_v_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/maximum_v'
    });

    ballhandle_dis_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/ballhandle_dis'
    });

     ballhandle_ang_param = new ROSLIB.Param({
        ros: Robot_Ctl,
        name: 'core/ballhandle_ang'
    });
}

function get_dynamic_parameter(){
    //setTimeout(get_dynamic_parameter, 1000);
    role_param.get(function(value) {
        document.getElementById("role").value = value;
    });
    gamestrat_param.get(function(value) {
        document.getElementById("game_start").checked = value;
    });
    chase_straight_param.get(function(value) {
        document.getElementById("chase_straight").checked = value;
    });
    Accelerate_param.get(function(value) {
        document.getElementById("Accelerate").checked = value;
    });
    ball_pwm_param.get(function(value) {
        document.getElementById("ball_pwm").checked = value;
    });
    shooting_start_param.get(function(value) {
        document.getElementById("shooting_start").checked = value;
    });
    game_state_param.get(function(value) {
        document.getElementById("game_state").value = value;
    });
    run_point_param.get(function(value) {
        document.getElementById("run_point").value = value;
    });
    run_x_param.get(function(value) {
        document.getElementById("run_xSlider").value = value;
        document.getElementById("run_xInput").value = value;
    });
    run_y_param.get(function(value) {
        document.getElementById("run_ySlider").value = value;
        document.getElementById("run_yInput").value = value;
    });
    run_yaw_param.get(function(value) {
        document.getElementById("run_yawSlider").value = value;
        document.getElementById("run_yawInput").value = value;
    });
    our_side_param.get(function(value) {
        document.getElementById("our_side").value = value;
        //console.log(Robot_Ctl);
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
    });
    attack_mode_param.get(function(value) {
        document.getElementById("attack_mode").value = value;
    });
    strategy_mode_param.get(function(value) {
        document.getElementById("strategy_mode").value = value;
    });
    orb_attack_ang_param.get(function(value) {
        document.getElementById("orb_attack_angSlider").value = value;
        document.getElementById("orb_attack_angInput").value = value;
    });
    atk_shoot_ang_param.get(function(value) {
        document.getElementById("atk_shoot_angSlider").value = value;
        document.getElementById("atk_shoot_angInput").value = value;
    });
    atk_shoot_dis_param.get(function(value) {
        document.getElementById("atk_shoot_disSlider").value = value;
        document.getElementById("atk_shoot_disInput").value = value;
    });
    minimum_w_param.get(function(value) {
        document.getElementById("minimum_wSlider").value = value;
        document.getElementById("minimum_wInput").value = value;
    });
    maximum_w_param.get(function(value) {
        document.getElementById("maximum_wSlider").value = value;
        document.getElementById("maximum_wInput").value = value;
    });
    minimum_v_param.get(function(value) {
        document.getElementById("minimum_vSlider").value = value;
        document.getElementById("minimum_vInput").value = value;
    });
    maximum_v_param.get(function(value) {
        document.getElementById("maximum_vSlider").value = value;
        document.getElementById("maximum_vInput").value = value;
    });
    ballhandle_dis_param.get(function(value) {
        document.getElementById("ballhandle_disSlider").value = value;
        document.getElementById("ballhandle_disInput").value = value;
    });
    ballhandle_ang_param.get(function(value) {
        document.getElementById("ballhandle_angSlider").value = value;
        document.getElementById("ballhandle_angInput").value = value;
    });
}