var monitor;

function MonitorSwitch(checked) {
    var video = document.getElementById("MapCanvas");
    var check = document.getElementById("CameraSwitch").checked;

    if (checked == true) {
        ViewButton(64);
        if (ChooseRobot == 1) {
            video.src = "http://" + document.getElementById("RobotIP").value + ":8080/stream?topic=/camera/image_monitor";
            //console.log(11);
        }
        if (ChooseRobot == 2) {
            video.src = "http://" + document.getElementById("RobotIP2").value + ":8080/stream?topic=/camera/image_monitor";
            //console.log(22);
        }
        if (ChooseRobot == 3) {
            video.src = "http://" + document.getElementById("RobotIP3").value + ":8080/stream?topic=/camera/image_monitor";
            //console.log(33);
        }
    } else {
        ViewButton(1);
        video.src = "img/Ground.png";
        // console.log(00);
    }
}