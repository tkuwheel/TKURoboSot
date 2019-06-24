
function ChooseShowFlag(num) {
    ShowFlag = num;
}

function SaveShowValue(){
    var ShowBox1 = [];
    $("[name=ChooseShowElement1]").each(function() {
        ShowBox1.push(parseFloat($(this).val()));
    });
    localStorage.setItem("RobotShow1", JSON.stringify(ShowBox1));
}

function Robot_Show() {
    var ShowBox = [];
    var speed = document.getElementById("SpeedInput").value;
    var vec3 = 0;
    if (ShowFlag == 0) {
        StandBy();
    } else if (ShowFlag == 1) {
        console.log('strat 1 show');
        obj = document.getElementsByName("ChooseShowElement1");
        for (var i = 0; i < obj.length; i++) {
            ShowBox.push(parseFloat(obj[i].value) * 1000);
        }
        console.log(ShowBox);
        ShowTime = 0;
        vec3 = new ROSLIB.Message({
            x: 0,
            y: 1 * speed,
            z: 0
        });
        setTimeout(CheckShow, 0, vec3);
        vec3 = new ROSLIB.Message({
            x: 1 * speed / 1.5,
            y: 1 * speed / 1.5,
            z: 0
        });
        setTimeout(CheckShow, ShowBox[0], vec3);
        vec3 = new ROSLIB.Message({
            x: 0,
            y: 1 * speed,
            z: 0
        });
        setTimeout(CheckShow, ShowBox[0] + ShowBox[1], vec3);
        vec3 = new ROSLIB.Message({
            x: 0,
            y: 0,
            z: -5
        });
        setTimeout(CheckShow, ShowBox[0] + ShowBox[1] + ShowBox[2], vec3);
        vec3 = new ROSLIB.Message({
            x: -(1 * speed * 0.866 ),
            y: 1 * speed / 2 ,
            z: 0
        });
        setTimeout(CheckShow, ShowBox[0] + ShowBox[1]+ShowBox[2]+ ShowBox[3], vec3);
        vec3 = new ROSLIB.Message({
            x: 0,
            y: 1 * speed,
            z: 0
        });
        setTimeout(CheckShow, ShowBox[0] + ShowBox[1] + ShowBox[2] + ShowBox[3]+ ShowBox[4], vec3);
        vec3 = new ROSLIB.Message({
            x: 0,
            y: 0,
            z: 5
        });
        setTimeout(CheckShow, ShowBox[0] + ShowBox[1] + ShowBox[2] + ShowBox[3]+ ShowBox[4]+ShowBox[5], vec3);
        vec3 = new ROSLIB.Message({
            x: 1 * speed / 1.5,
            y: 1 * speed / 1.5,
            z: 0
        });
        setTimeout(CheckShow,ShowBox[0]+ShowBox[1]+ShowBox[2]+ ShowBox[3] + ShowBox[4]+ ShowBox[5]+ ShowBox[6],vec3);
        setTimeout(initial, ShowBox[0] + ShowBox[1] + ShowBox[2] + ShowBox[3] + ShowBox[4] + ShowBox[5]+ ShowBox[6]+ ShowBox[7]);
    }

}

function CheckShow(vec3) {
    console.log(vec3);
    if (ShowTime == 0) {
        PublishTopicCmdVel(vec3);
        ShowTime += 1;
        setTimeout(CheckShow, 50, vec3);
    } else if (ShowTime == 1) {
        PublishTopicCmdVel(vec3);
        ShowTime = 0;
    }
}

function initial() {
    var vec3 = new ROSLIB.Message({
        x: 0,
        y: 0,
        z: 0
    });
    console.log(vec3);
    PublishTopicCmdVel(vec3);
}
