$(function($) {
    $("#HoldBallDiv1").click(function() {
        if (this.value) {
            $("#HoldBallButton1").remove();
            var holdball_i = $(document.createElement('i'))
                .attr("class", "fa fa-futbol-o fa-3x")
                .attr("aria-hidden", "true")
                .attr("id", "HoldBallButton1")
            holdball_i.appendTo("#HoldBallDiv1");
            this.value = 0;
            HoldBallSwitch(0,1);
        } else {
            $("#HoldBallButton1").remove();
            var holdball_i = $(document.createElement('i'))
                .attr("class", "fa fa-futbol-o fa-3x fa-spin")
                .attr("aria-hidden", "true")
                .attr("id", "HoldBallButton1")
                .attr("style","color: #FFCC00;")
            holdball_i.appendTo("#HoldBallDiv1");
            this.value = 1;
            HoldBallSwitch(1,1);
        }
    });
});



