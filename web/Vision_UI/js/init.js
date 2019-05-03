$("input:radio").attr("checked", false);

document.getElementById("CameraButton").checked = true;
document.getElementById("HSVButton").checked = true;
setTimeout(topicROSParameterButton,1000,1);
service_connect();
