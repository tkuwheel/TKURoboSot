//
function ToInputValue(newValue, name, num) {
    document.getElementsByName(name)[num].value = newValue;
}

function ToSliderValue(newValue, name, num) {
    if (newValue > 100) {
        newValue = 100;
    }
    document.getElementsByName(name)[1].value = newValue;
    document.getElementsByName(name)[num].value = newValue;
}
//======================================================================
function JudgeShootInput(name, value) {
    if (value > 100)
        value = 100;
    document.getElementsByName(name)[0].value = value;
}