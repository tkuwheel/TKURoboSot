import rospy

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