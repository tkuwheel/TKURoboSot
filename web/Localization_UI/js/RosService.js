var updateClient = new ROSLIB.Service({
    ros: ros,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});
var updateClient2 = new ROSLIB.Service({
    ros: ros2,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});
var updateClient3 = new ROSLIB.Service({
    ros: ros3,
    name: '/StrategyParam',
    serviceType: 'param_convey/strategy_param'
});

var request = new ROSLIB.ServiceRequest({
    receive: 1
});

function up() {
    console.log('updating parameter');
    updateClient.callService(request, function(res) {
        if (res.update == 2) {
            console.log('Parameter is saved');
        }
    });

    updateClient2.callService(request, function(res) {
        if (res.update == 2) {
            console.log('Parameter is saved');
        }
    });

    updateClient3.callService(request, function(res) {
        if (res.update == 2) {
            console.log('Parameter is saved');
        }
    });
}