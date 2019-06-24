//ROS_connect
var light, light2, light3;
var light = document.getElementById("Light");
var light2 = document.getElementById("Light2");
var light3 = document.getElementById("Light3");

if (typeof(Storage) !== "undefined") {
    if (localStorage.getItem("IP1") != null) {
        document.getElementById("RobotIP").value = localStorage.getItem("IP1");
    } else {
        document.getElementById("RobotIP").value = "localhost";
        localStorage.IP1 = "localhost";
    }
    if (localStorage.getItem("Host") != null) {
        document.getElementById("RobotHost").value = localStorage.getItem("Host");
    } else {
        document.getElementById("RobotHost").value = "9090";
        localStorage.Host = "9090"
    }

    if (localStorage.getItem("IP2") != null) {
        document.getElementById("RobotIP2").value = localStorage.getItem("IP2");
    } else {
        document.getElementById("RobotIP2").value = "localhost";
        localStorage.IP2 = "localhost";
    }
    if (localStorage.getItem("Host2") != null) {
        document.getElementById("RobotHost2").value = localStorage.getItem("Host2");
    } else {
        document.getElementById("RobotHost2").value = "9090";
        localStorage.Host2 = "9090"
    }

    if (localStorage.getItem("IP3") != null) {
        document.getElementById("RobotIP3").value = localStorage.getItem("IP3");
    } else {
        document.getElementById("RobotIP3").value = "localhost";
        localStorage.IP3 = "localhost";
    }
    if (localStorage.getItem("Host3") != null) {
        document.getElementById("RobotHost3").value = localStorage.getItem("Host3");
    } else {
        document.getElementById("RobotHost3").value = "9090";
        localStorage.Host3 = "9090"
    }
} else {
    console.log('Sorry, your browser does not support Web Storage...');
}


//Robot_connnet
var ros = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP").value + ':' + document.getElementById("RobotHost").value
});
var ros2 = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP2").value + ':' + document.getElementById("RobotHost2").value
});
var ros3 = new ROSLIB.Ros({
    url: 'ws://' + document.getElementById("RobotIP3").value + ':' + document.getElementById("RobotHost3").value
});

//confirm_connect
ros.on('connection', function() {
    console.log('Robot1 Connected to websocket server.');
    light = "connected";
    document.getElementById("Light").src = "img/light1.png";
    CheckIP[0] = 1;
});
ros.on('error', function(error) {
    console.log('Robot1 Error connecting to websocket server:');
    light = "disconnected";
    document.getElementById("Light").src = "img/light2.png";
    CheckIP[0] = 0;
});
ros.on('close', function() {
    console.log('Robot1 Connection to websocket server closed.');
    light = "disconnected";
    document.getElementById("Light").src = "img/light2.png";
    CheckIP[0] = 0;
});

ros2.on('connection', function() {
    console.log('Robot2 Connected to websocket server.');
    light2 = "connected";
    document.getElementById("Light2").src = "img/light1.png"
    CheckIP[1] = 1;
});
ros2.on('error', function(error) {
    console.log('Robot2 Error connecting to websocket server:');
    light2 = "disconnected";
    document.getElementById("Light2").src = "img/light2.png"
    CheckIP[1] = 0;
});
ros2.on('close', function() {
    console.log('Robot2 Connection to websocket server closed.');
    light2 = "disconnected";
    document.getElementById("Light2").src = "img/light2.png"
    CheckIP[1] = 0;
});

ros3.on('connection', function() {
    console.log('Robot3 Connected to websocket server.');
    light3 = "connected";
    document.getElementById("Light3").src = "img/light1.png"
    CheckIP[2] = 1;
});
ros3.on('error', function(error) {
    console.log('Robot3 Error connecting to websocket server:');
    light3 = "disconnected";
    document.getElementById("Light3").src = "img/light2.png"
    CheckIP[2] = 0;
});
ros3.on('close', function() {
    console.log('Robot3 Connection to websocket server closed.');
    light3 = "disconnected";
    document.getElementById("Light3").src = "img/light2.png"
    CheckIP[2] = 0;
});

//When Connect botton is pressed
function RobotConnect() {
    var IP1 = document.getElementById("RobotIP").value;
    if (IP1 != '') {
        localStorage.IP1 = IP1;
    } else {
        if (localStorage.getItem("IP1") != null) {
            IP1 = localStorage.getItem("IP1");
        } else {
            IP1 = "localhost";
            localStorage.IP1 = "localhost";
        }
    }
    var IP2 = document.getElementById("RobotIP2").value;
    if (IP2 != '') {
        localStorage.IP2 = IP2;
    } else {
        if (localStorage.getItem("IP2") != null) {
            IP2 = localStorage.getItem("IP2");
        } else {
            IP2 = "localhost";
            localStorage.IP2 = "localhost";
        }
    }
    var IP3 = document.getElementById("RobotIP3").value;
    if (IP3 != '') {
        localStorage.IP3 = IP3;
    } else {
        if (localStorage.getItem("IP3") != null) {
            IP3 = localStorage.getItem("IP3");
        } else {
            IP3 = "localhost";
            localStorage.IP3 = "localhost";
        }
    }

    var Host = document.getElementById("RobotHost").value;
    if (Host != '') {
        localStorage.Host = Host;
    } else {
        if (localStorage.getItem("Host") != null) {
            Host = localStorage.getItem("Host");
        } else {
            Host = "9090";
            localStorage.Host = "9090";
        }
    }
    var Host2 = document.getElementById("RobotHost2").value;
    if (Host2 != '') {
        localStorage.Host2 = Host2;
    } else {
        if (localStorage.getItem("Host2") != null) {
            Host2 = localStorage.getItem("Host2");
        } else {
            Host2 = "9090";
            localStorage.Host2 = "9090";
        }
    }
    var Host3 = document.getElementById("RobotHost3").value;
    if (Host3 != '') {
        localStorage.Host3 = Host3;
    } else {
        if (localStorage.getItem("Host3") != null) {
            Host3 = localStorage.getItem("Host3");
        } else {
            Host = "9090";
            localStorage.Host3 = "9090";
        }
    }

    ros = new ROSLIB.Ros({
        url: 'ws://' + IP1 + ':' + Host
    });
    ros2 = new ROSLIB.Ros({
        url: 'ws://' + IP2 + ':' + Host2
    });
    ros3 = new ROSLIB.Ros({
        url: 'ws://' + IP3 + ':' + Host3
    });

    //confirm_connect
    ros.on('connection', function() {
        console.log('Robot1 Connected to websocket server.');
        light = "connected";
        document.getElementById("Light").src = "img/light1.png"
        CheckIP[0] = 1;
    });
    ros.on('error', function(error) {
        console.log('Robot1 Error connecting to websocket server:');
        light = "disconnected";
        document.getElementById("Light").src = "img/light2.png"
        CheckIP[0] = 0;
    });
    ros.on('close', function() {
        console.log('Robot1 Connection to websocket server closed.');
        light = "disconnected";
        document.getElementById("Light").src = "img/light2.png"
        CheckIP[0] = 0;
    });

    ros2.on('connection', function() {
        console.log('Robot2 Connected to websocket server.');
        light2 = "connected";
        document.getElementById("Light2").src = "img/light1.png"
        CheckIP[1] = 1;
    });
    ros2.on('error', function(error) {
        console.log('Robot2 Error connecting to websocket server:');
        light2 = "disconnected";
        document.getElementById("Light2").src = "img/light2.png"
        CheckIP[1] = 0;
    });
    ros2.on('close', function() {
        console.log('Robot2 Connection to websocket server closed.');
        light2 = "disconnected";
        document.getElementById("Light2").src = "img/light2.png"
        CheckIP[1] = 0;
    });

    ros3.on('connection', function() {
        console.log('Robot3 Connected to websocket server.');
        light3 = "connected";
        document.getElementById("Light3").src = "img/light1.png"
        CheckIP[2] = 1;
    });
    ros3.on('error', function(error) {
        console.log('Robot3 Error connecting to websocket server:');
        light3 = "disconnected";
        document.getElementById("Light3").src = "img/light2.png"
        CheckIP[2] = 0;
    });
    ros3.on('close', function() {
        console.log('Robot3 Connection to websocket server closed.');
        light3 = "disconnected";
        document.getElementById("Light3").src = "img/light2.png"
        CheckIP[2] = 0;
    });
    SetParamRobotNum();
    up();
}

function RobotCloseConnect() {
    ros.close();
    ros2.close();
    ros3.close();
}
