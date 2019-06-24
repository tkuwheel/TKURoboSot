//service
// -----------------

var updateClient = new ROSLIB.Service({
    ros: ros,
    name: '/StrategyParam',
    serviceType: 'vision/checkpoint'
});
/*var updateClient2 = new ROSLIB.Service({
    ros: ros2,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});
var updateClient3 = new ROSLIB.Service({
    ros: ros3,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});*/

var request = new ROSLIB.ServiceRequest({
    receive: 1
});

function up() {
    document.getElementById("Update").style.cursor = "wait";
    context4.fillStyle = "yellow";
    context4.fill();

    updateClient.callService(request, function(res) {
        if (res.update == 2) {
            document.getElementById("Update").style.cursor = "default";
            context4.fillStyle = "green";
            context4.fill();
        }
    });

    updateClient2.callService(request, function(res) {
        if (res.update == 2) {
            document.getElementById("Update").style.cursor = "default";
            context4.fillStyle = "green";
            context4.fill();
        }
    });

    updateClient3.callService(request, function(res) {
        if (res.update == 2) {
            document.getElementById("Update").style.cursor = "default";
            context4.fillStyle = "green";
            context4.fill();
        }
    });
}
