$(document).ready(function() {

    $('#DistanceSlider').change(function() {
        var space = document.getElementsByName('DistanceElement1')[0].value;
        var counter = document.getElementsByName('DistanceElement1')[1].value;
        var i;

        for (i = OldCounter; i > 0; i--) {
            $("#TextBoxDiv" + i).remove();
        }
        OldCounter = counter;
        for (i = 1; i < counter; i++) {
            var newTextBoxDiv = $(document.createElement('div'))
                .attr("id", 'TextBoxDiv' + i);

            newTextBoxDiv.appendTo("#DistanceSetting");

            newTextBoxDiv.after().html('<table width="100%" height="100%"> <tr> <td>'
                                       +'<label class="Distance_font">' 
                                       + '<input type="radio" name="Distanceradio" class="myCheck">' 
                                       + 'Distance:'
                                       + (i*space)
                                       + 'cm </label> </td> <td>'
                                       +'<input size="10" type="text" disabled="true" '
                                       + 'name="DistanceElement2">'
                                       + '</td>');

            newTextBoxDiv.appendTo("#DistanceSetting");
        }
    });
})
