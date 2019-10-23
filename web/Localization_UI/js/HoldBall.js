function holdball(){
    //console.log("hollball",this.getAttribute('value'));  
    if (parseInt(document.getElementById("HoldBallDiv1").getAttribute('value'))) {
        $("#HoldBallButton1").remove();
        var holdball_i = $(document.createElement('i'))
            .attr("class", "fa fa-futbol-o fa-2x")
            .attr("aria-hidden", "true")
            .attr("id", "HoldBallButton1")
        holdball_i.appendTo("#HoldBallDiv1");
        document.getElementById("HoldBallDiv1").setAttribute('value',0);
        HoldBallSwitch(0,1);
    } else {
        $("#HoldBallButton1").remove();
        var holdball_i = $(document.createElement('i'))
            .attr("class", "fa fa-futbol-o fa-2x fa-spin")
            .attr("aria-hidden", "true")
            .attr("id", "HoldBallButton1")
            .attr("style","color: #FFCC00;")
        holdball_i.appendTo("#HoldBallDiv1");
        document.getElementById("HoldBallDiv1").setAttribute('value',1);
        HoldBallSwitch(1,1);
    }
}

$(function($) {
    $("#HoldBallDiv1").click(function() {
        holdball();
    });
});

