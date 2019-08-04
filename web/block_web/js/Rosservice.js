    var myBoolean1 = new Boolean();
    var myBoolean2 = new Boolean();
    var myBoolean3 = new Boolean();
    var myBoolean4 = new Boolean()

    function processFormData() {
         if(document.getElementById("locate").checked) {
           myBoolean3 = Boolean(1);
         }else{
           myBoolean3 = Boolean(0)
         }
         if(document.getElementById("ball_pwm").checked) {
           myBoolean4 = Boolean(1);
         }else{
           myBoolean4 = Boolean(0)
         }

        

        var locate = new ROSLIB.Message({
            data:  myBoolean3
        });
        var ball_pwm = new ROSLIB.Message({
            data:  myBoolean4
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

        var our_side = new ROSLIB.Message({
        data: document.getElementById("our_side").value
        });




        var request = new ROSLIB.ServiceRequest({
        config: {
            bools: [
                {name: 'game_start', value: game_start.data},
                {name: 'locate', value: locate.data},
                {name: 'ball_pwm', value: ball_pwm.data},
            ],
            ints: [
                {name: 'ballhandle_dis', value: 0},
                {name: 'ballhandle_ang', value: 0},
            ],
            strs: [
                {name: 'game_state', value: game_state.data},
                {name: 'our_side', value: our_side.data},
            ],
            doubles: [
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
