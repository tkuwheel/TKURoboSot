    var myBoolean1 = new Boolean();
    var myBoolean2 = new Boolean();
    var myBoolean3 = new Boolean();
    var myBoolean4 = new Boolean();
    var myBoolean5 = new Boolean()

    function processFormData() {
        if(document.getElementById("chase_straight").checked) {
           myBoolean2 = Boolean(1);
        }else{
           myBoolean2 = Boolean(0)
         }
         if(document.getElementById("Accelerate").checked) {
           myBoolean3 = Boolean(1);
         }else{
           myBoolean3 = Boolean(0)
         }
         if(document.getElementById("ball_pwm").checked) {
           myBoolean4 = Boolean(1);
         }else{
           myBoolean4 = Boolean(0)
         }
         if(document.getElementById("shooting_start").checked) {
           myBoolean5 = Boolean(1);
         }else{
           myBoolean5 = Boolean(0)
         }

        
        var chase_straight = new ROSLIB.Message({
            data:  myBoolean2
        });
        var Accelerate = new ROSLIB.Message({
            data:  myBoolean3
        });
        var ball_pwm = new ROSLIB.Message({
            data:  myBoolean4
        });
        var shooting_start = new ROSLIB.Message({
            data:  myBoolean5
        });



        var ballhandle_dis = new ROSLIB.Message({
            data: parseInt(document.getElementById("ballhandle_disInput").value)
        });
        var ballhandle_ang = new ROSLIB.Message({
            data: parseInt(document.getElementById("ballhandle_angInput").value)
        });



        var orb_attack_ang = new ROSLIB.Message({
        data: parseFloat(document.getElementById("orb_attack_angInput").value)
        });
        var atk_shoot_ang = new ROSLIB.Message({
        data: parseFloat(document.getElementById("atk_shoot_angInput").value)
        });
        var atk_shoot_dis = new ROSLIB.Message({
        data: parseFloat(document.getElementById("atk_shoot_disInput").value)
        });
        var minimum_w = new ROSLIB.Message({
        data: parseFloat(document.getElementById("minimum_wInput").value)
        });
        var maximum_w = new ROSLIB.Message({
        data: parseFloat(document.getElementById("maximum_wInput").value)
        });
        var minimum_v = new ROSLIB.Message({
        data: parseFloat(document.getElementById("minimum_vInput").value)
        });
        var maximum_v = new ROSLIB.Message({
        data: parseFloat(document.getElementById("maximum_vInput").value)
        });
        var run_x = new ROSLIB.Message({
        data: parseFloat(document.getElementById("run_xInput").value)
        });
        var run_y = new ROSLIB.Message({
        data: parseFloat(document.getElementById("run_yInput").value)
        });
        var run_yaw = new ROSLIB.Message({
        data: parseFloat(document.getElementById("run_yawInput").value)
        });

        var game_state = new ROSLIB.Message({
        data: document.getElementById("game_state").value
        });
        var strategy_mode = new ROSLIB.Message({
        data: document.getElementById("strategy_mode").value
        });
        var attack_mode = new ROSLIB.Message({
        data: document.getElementById("attack_mode").value
        });
        var our_side = new ROSLIB.Message({
        data: document.getElementById("our_side").value
        });
        var run_point = new ROSLIB.Message({
        data: document.getElementById("run_point").value
        });



        var request = new ROSLIB.ServiceRequest({
        config: {
            bools: [
                {name: 'game_start', value: game_start.data},
                {name: 'chase_straight', value: chase_straight.data},
                {name: 'Accelerate', value: Accelerate.data},
                {name: 'ball_pwm', value: ball_pwm.data},
                {name: 'shooting_start', value: shooting_start.data},
            ],
            ints: [
                {name: 'ballhandle_dis', value: ballhandle_dis.data},
                {name: 'ballhandle_ang', value: ballhandle_ang.data},
            ],
            strs: [
                {name: 'game_state', value: game_state.data},
                {name: 'strategy_mode', value: strategy_mode.data},
                {name: 'attack_mode', value: attack_mode.data},
                {name: 'our_side', value: our_side.data},
                {name: 'run_point', value: run_point.data},
            ],
            doubles: [
                {name: 'orb_attack_ang', value: orb_attack_ang.data},
                {name: 'atk_shoot_ang', value: atk_shoot_ang.data},
                {name: 'atk_shoot_dis', value: atk_shoot_dis.data},
                {name: 'minimum_w', value: minimum_w.data},
                {name: 'minimum_w', value: minimum_w.data},
                {name: 'maximum_w', value: maximum_w.data},
                {name: 'minimum_v', value: minimum_v.data},
                {name: 'maximum_v', value: maximum_v.data},
                {name: 'run_x', value: run_x.data},
                {name: 'run_y', value: run_y.data},
                {name: 'run_yaw', value: run_yaw.data},
            ],
            groups: [
                // {name: '', state: false, id: 0, parent: 0}
            ]
        }

        });

        dynaRecClient.callService(request, function(result) {
        console.log('Result for service call on '
            + dynaRecClient.name
            + ': '
            + JSON.stringify(result, null, 2));
        });
        
    }

    function gameStart(){
        if(document.getElementById("game_start").checked) {
            myBoolean1 = Boolean(1);
         }else{
            myBoolean1 = Boolean(0)
         }
        var game_start = new ROSLIB.Message({
            data:  myBoolean1
        });
        
         var request = new ROSLIB.ServiceRequest({
        config: {
            bools: [
                {name: 'game_start', value: game_start.data},
               
            ]
           
        }

        });
        dynaRecClient.callService(request, function(result) {
        console.log('Result for service call on '
            + dynaRecClient.name
            + ': '
            + JSON.stringify(result, null, 2));
        });
    }

    function ToInputValue(newValue, name, num) {
    document.getElementsByName(name)[num].value = newValue;
            }

function ToSliderValue(newValue, name, num) {
    /*if (newValue > 100) {
        newValue = 100;
    }*/
    document.getElementsByName(name)[1].value = newValue;
    document.getElementsByName(name)[num].value = newValue;
}
//======================================================================
