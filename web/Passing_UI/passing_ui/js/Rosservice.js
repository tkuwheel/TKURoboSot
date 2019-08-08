    var myBoolean1 = new Boolean();
    var myBoolean2 = new Boolean();
    var myBoolean3 = new Boolean();
    var myBoolean4 = new Boolean();
    var myBoolean5 = new Boolean();
    var myBoolean6 = new Boolean();


    function processFormData() {
        if(document.getElementById("target_vision_red").checked) {
           myBoolean2 = Boolean(1);
        }else{
           myBoolean2 = Boolean(0)
         }
         if(document.getElementById("target_vision_yellow").checked) {
           myBoolean3 = Boolean(1);
         }else{
           myBoolean3 = Boolean(0)
         }
         if(document.getElementById("target_vision_blue").checked) {
           myBoolean4 = Boolean(1);
         }else{
           myBoolean4 = Boolean(0)
         }
         if(document.getElementById("target_vision_white").checked) {
           myBoolean5 = Boolean(1);
         }else{
           myBoolean5 = Boolean(0)
         }
         if(document.getElementById("passing_power").checked) {
            myBoolean6 = Boolean(1);
          }else{
            myBoolean6 = Boolean(0)
          }

        
        var target_vision_red = new ROSLIB.Message({
            data:  myBoolean2
        });
        var target_vision_yellow = new ROSLIB.Message({
            data:  myBoolean3
        });
        var target_vision_blue = new ROSLIB.Message({
            data:  myBoolean4
        });
        var target_vision_white = new ROSLIB.Message({
            data:  myBoolean5
        });
        var passing_power = new ROSLIB.Message({
            data:  myBoolean6
        });


        var ballhandle_dis = new ROSLIB.Message({
            data: parseInt(document.getElementById("ballhandle_disInput").value)
        });
        var ballhandle_ang = new ROSLIB.Message({
            data: parseInt(document.getElementById("ballhandle_angInput").value)
        });
        var padding_ball = new ROSLIB.Message({
            data: parseInt(document.getElementById("padding_ballInput").value)
        });
        var padding_target = new ROSLIB.Message({
            data: parseInt(document.getElementById("padding_targetInput").value)
        });
       
        



        var orb_attack_ang = new ROSLIB.Message({
        data: parseFloat(document.getElementById("orb_attack_angInput").value)
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
        var adjust_ball1_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball1_xInput").value)
        });
        var adjust_ball1_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball1_yInput").value)
        });
        var adjust_ball2_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball2_xInput").value)
        });
        var adjust_ball2_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball2_yInput").value)
        });
        var adjust_ball3_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball3_xInput").value)
        });
        var adjust_ball3_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball3_yInput").value)
        });
        var adjust_ball4_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball4_xInput").value)
        });
        var adjust_ball4_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_ball4_yInput").value)
        });
        var adjust_target1_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target1_xInput").value)
        });
        var adjust_target1_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target1_yInput").value)
        });
        var adjust_target2_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target2_xInput").value)
        });
        var adjust_target2_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target2_yInput").value)
        });
        var adjust_target3_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target3_xInput").value)
        });
        var adjust_target3_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target3_yInput").value)
        });   
        var adjust_target4_x = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target4_xInput").value)
        });   
        var adjust_target4_y = new ROSLIB.Message({
        data: parseInt(document.getElementById("adjust_target4_yInput").value)
        });   
            
        

        
        var strategy_mode = new ROSLIB.Message({
        data: document.getElementById("strategy_mode").value
        });

       



        var request = new ROSLIB.ServiceRequest({
        config: {
            bools: [
                {name: 'game_start', value: game_start.data},
                {name: 'using_orbit', value: using_orbit.data},
                {name: 'target_vision_red', value: target_vision_red.data},
                {name: 'target_vision_yellow', value: target_vision_yellow.data},
                {name: 'target_vision_blue', value: target_vision_blue.data},
                {name: 'target_vision_white', value: target_vision_white.data},
                {name: 'passing_power', value: passing_power.data},
            ],
            ints: [
                {name: 'ballhandle_dis', value: ballhandle_dis.data},
                {name: 'ballhandle_ang', value: ballhandle_ang.data},
                {name: 'padding_ball', value: padding_ball.data},
                {name: 'padding_target', value: padding_target.data},
                
                {name: 'adjust_ball1_x', value: adjust_ball1_x.data},
                {name: 'adjust_ball1_y', value: adjust_ball1_y.data},
                {name: 'adjust_ball2_x', value: adjust_ball2_x.data},
                {name: 'adjust_ball2_y', value: adjust_ball2_y.data},
                {name: 'adjust_ball3_x', value: adjust_ball3_x.data},
                {name: 'adjust_ball3_y', value: adjust_ball3_y.data},
                {name: 'adjust_ball4_x', value: adjust_ball4_x.data},
                {name: 'adjust_ball4_y', value: adjust_ball4_y.data},
                {name: 'adjust_target1_x', value: adjust_target1_x.data},
                {name: 'adjust_target1_y', value: adjust_target1_y.data},
                {name: 'adjust_target2_x', value: adjust_target2_x.data},
                {name: 'adjust_target2_y', value: adjust_target2_y.data},
                {name: 'adjust_target3_x', value: adjust_target3_x.data},
                {name: 'adjust_target3_y', value: adjust_target3_y.data},
                {name: 'adjust_target4_x', value: adjust_target4_x.data},
                {name: 'adjust_target4_y', value: adjust_target4_y.data},
            ],
            strs: [
                {name: 'strategy_mode', value: strategy_mode.data},
                {name: 'level', value: level.data},
            ],
            doubles: [
                {name: 'orb_attack_ang', value: orb_attack_ang.data},
                {name: 'minimum_w', value: minimum_w.data},
                {name: 'minimum_w', value: minimum_w.data},
                {name: 'maximum_w', value: maximum_w.data},
                {name: 'minimum_v', value: minimum_v.data},
                {name: 'maximum_v', value: maximum_v.data},
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
