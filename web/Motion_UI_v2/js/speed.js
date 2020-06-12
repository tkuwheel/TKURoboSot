var SPEED = document.getElementById("SpeedInput").value;
var MAX_SPEED = document.getElementById("max_speed_input").value;
var SPEED_TOPIC = document.getElementById("topic_name_input").value;
var COORD;
var default_speed = 5;
var default_max_speed = 100;
var default_speed_topic = "motion/cmd_vel";
var default_coord = 1;
if (typeof(Storage) !== "undefined") {
    if (localStorage.getItem("SPEED") != null) {
        document.getElementById("SpeedInput").value = localStorage.getItem("SPEED");
    } else {
        document.getElementById("SpeedInput").value = default_speed;
        localStorage.SPEED = default_speed;
    }
    if (localStorage.getItem("SPEED") != null) {
        document.getElementById("SpeedSlider").value = localStorage.getItem("SPEED");
    } else {
        document.getElementById("SpeedSlider").value = default_speed;
        localStorage.SPEED = default_speed;
    }

    if (localStorage.getItem("MAX_SPEED") != null) {
        document.getElementById("max_speed_input").value = localStorage.getItem("MAX_SPEED");
    } else {
        document.getElementById("max_speed_input").value = default_max_speed;
        localStorage.MAX_SPEED = default_max_speed;
    }
    set_max_speed(localStorage.MAX_SPEED);
    if (localStorage.getItem("SPEED_TOPIC") != null) {
        document.getElementById("topic_name_input").value = localStorage.getItem("SPEED_TOPIC");
    } else {
        document.getElementById("topic_name_input").value = default_speed_topic;
        localStorage.SPEED_TOPIC = default_speed_topic;
    }
    if (localStorage.getItem("COORD") != null) {
        if(localStorage.getItem("COORD")==1){
            $('#coord1input').prop('checked',true);
            $('#coord1input').change();
            $('#coord2input').change();
        }
        if(localStorage.getItem("COORD")==2){
            $('#coord2input').prop('checked',true);
            $('#coord1input').change();
            $('#coord2input').change();
        }
    } else {
        localStorage.COORD = default_coord;
        if(localStorage.getItem("COORD")==1){
            $('#coord1input').prop('checked',true);
            $('#coord1input').change();
            $('#coord2input').change();
        }
        if(localStorage.getItem("COORD")==2){
            $('#coord2input').prop('checked',true);
            $('#coord1input').change();
            $('#coord2input').change();
        }
    }
} else {
    console.log('Sorry, your browser does not support Web Storage...');
}
function set_speed_topic(value){
   SPEED_TOPIC = document.getElementById("topic_name_input").value;
   localStorage.SPEED_TOPIC = SPEED_TOPIC;
   console.log(value);

   if(SPEED_TOPIC=="cmd_vel"){
       if(document.getElementById("SpeedSlider").max>3){
           document.getElementById("max_speed_input").value=3;
           set_max_speed(3);
           set_speed(1);
       }
   }
   
}
function set_max_speed(value){
    if(value>=0 && value<=100){
        localStorage.MAX_SPEED = value;
        document.getElementById("max_speed").innerText = value;
        document.getElementById("SpeedSlider").max = value;
        if(value<=10){
            document.getElementById("SpeedSlider").step = 0.1;
        }else{
            document.getElementById("SpeedSlider").step = 1;
        }
        document.getElementById("SpeedInput").value = document.getElementById("SpeedSlider").value;
    }
}
function set_speed(value){
    if(value>=0 && value<=100){
        localStorage.SPEED = value;
    }
    document.getElementById("SpeedInput").value=value;
    document.getElementById("SpeedSlider").value=value;
}
function set_coord(value){
    localStorage.COORD = value;
}
