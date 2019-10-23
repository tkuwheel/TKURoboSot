function holdball(robotnumber){
    let robotbutton = null;
    let robotdiv = null;
    let hold_ball_checked = null;
    
    if (robotnumber == 1){
        robotbutton = "HoldBallButton1";
        robotdiv = "HoldBallDiv1";
    }
    if (robotnumber == 2){
        robotbutton = "HoldBallButton2";
        robotdiv = "HoldBallDiv2";
    }
    if (robotnumber == 3){
        robotbutton = "HoldBallButton3";
        robotdiv = "HoldBallDiv3";
    }
    if(robotbutton!=null){
        if (parseInt(document.getElementById(robotdiv).getAttribute('value'))) {
            $("#"+robotbutton).remove();
            var holdball_i = $(document.createElement('i'))
                .attr("class", "fa fa-futbol-o fa-3x")
                .attr("aria-hidden", "true")
                .attr("id", robotbutton)
            holdball_i.appendTo("#"+robotdiv);
            document.getElementById(robotdiv).setAttribute('value',0);
            HoldBallSwitch(0,robotnumber);
        } else {
            $("#"+robotbutton).remove();
            var holdball_i = $(document.createElement('i'))
                .attr("class", "fa fa-futbol-o fa-3x fa-spin")
                .attr("aria-hidden", "true")
                .attr("id", robotbutton)
                .attr("style","color: #FFCC00;")
            holdball_i.appendTo("#"+robotdiv);
            document.getElementById(robotdiv).setAttribute('value',1);
            HoldBallSwitch(1,robotnumber);
        }
    }
}

$(function($) {
    $("#HoldBallDiv1").click(function() {
        holdball(1);
    });
    $("#HoldBallDiv2").click(function() {
        holdball(2);
    });
    $("#HoldBallDiv3").click(function() {
        holdball(3);
    });
});
