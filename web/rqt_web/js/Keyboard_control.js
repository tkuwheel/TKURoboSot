window.addEventListener("keydown", keysdown, false);
window.addEventListener("keyup", keyuped, false);

var vec3;
var keys = [];

function keysdown(e){
    keys[e.keyCode] = true;
    
    if (keys[70]) {
        var items = document.getElementById("game_start");
        if (items.checked == true){
            items.checked = false;
        }
        else  items.checked = true;
        gameStart();
    }
    if (keys[71]) {
        var items = document.getElementById("change_plan");
        if (items.checked == true){
            items.checked = false;
        }
        else  items.checked = true;
        chg();
    }
    if (keys[68]) {
        processFormData();
    }
}
function keyuped(e) {
      
    keys[e.keyCode] = false;
    
}