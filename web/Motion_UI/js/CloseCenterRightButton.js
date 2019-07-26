function blockMenu(Evt) {
    // window.event 是IE才有的物件
    if (window.event) {
        Evt = window.event;
        Evt.returnValue = false; //取消IE預設事件
    } else
        Evt.preventDefault(); //取消DOM預設事件
}

function lock(Evt) {
    // window.event 是IE才有的物件
    if (Evt.type == "mousedown") {
        if (Evt.button == 1) {
            return false;
        }
        // 擋 IE 按下 F1 鈕時會觸發的 onhelp 事件
    }
}

document.oncontextmenu = blockMenu;
document.onmousedown = lock;
