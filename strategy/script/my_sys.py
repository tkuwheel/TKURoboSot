import rospy
import sys, getopt
import re
import time

def logInOne(msg):
  print(msg, end='\r')
  time.sleep(.5)

def log(msg, rosout = False, level = "INFO"):
  if rosout:
    if level == "ERROR":
      rospy.logerr(msg)
    elif level == "WARRING":
      rospy.logwarn(msg)
    else:
      rospy.loginfo(msg)
  else:
    print("\x1b[0;33;41m{}\x1b[0m".format(msg))

def SysCheck(argv):
  r = re.compile("-?-?s[im]?|-?-?h[elp]?|[tT][rR][uU][eE]")
  argL = list(filter(r.match, argv))
  if any("s" in s for s in [k.lower() for k in argL]):
    return "Simulative Mode"
  elif any("true" in s for s in [k.lower() for k in argL]):
    return "Simulative Mode"
  elif any("h" in  s for s in [k.lower() for k in argL]):
    log("Append argument --sim to start with simulative mode.")
    log("Try $ roslaunch strategy test.launch sim:=true")
    sys.exit()
  else:
    return "Native Mode"