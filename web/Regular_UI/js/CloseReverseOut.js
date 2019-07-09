 var omitformtags = ["input", "textarea", "select"]
 omitformtags = omitformtags.join("|")

 function disableselect(e) {
     if (omitformtags.indexOf(e.target.tagName.toLowerCase()) == -1)
         return false
 }

 function reEnable() {
     return true
 }

 if (typeof document.onselectstart != "undefined")
     document.onselectstart = new Function("return false")
 else {
     document.onmousedown = disableselect
     document.onmouseup = reEnable
 }
