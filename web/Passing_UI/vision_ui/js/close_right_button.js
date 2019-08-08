function blockMenu(Evt) {
        // window.event 是IE才有的物件
        if (window.event) {
            Evt = window.event;
            Evt.returnValue = false; //取消IE預設事件
        } else
            Evt.preventDefault(); //取消DOM預設事件
    }

document.oncontextmenu = blockMenu;

