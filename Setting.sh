#!/bin/bash
# Program:
#     This program will setting environment variable for ros namespace.
# History:
# 2019/3/19 NEET First Edit
# 2019/5/10 NEET Add Camera serial setting

SCRIPT_PATH=`cd "$( dirname ${BASH_SOURCE[0]})" && pwd`

if [ "${BASH_SOURCE[0]}" -ef "$0" ]
then
    echo "Hey, you should source this script, not execute it!"
    exit 1
fi

function Check() {
  if grep -q "source $SCRIPT_PATH/.robot_setting" "$HOME/.bashrc";
  then
    echo "OK"
  else
    echo "source $SCRIPT_PATH/.robot_setting >> ~/.bashrc"
    echo "source $SCRIPT_PATH/.robot_setting" >> ~/.bashrc
  fi
}

function Change() {
  echo $1
  if [ $1 == "CHANGE_ROBOT_NUMBER" ]
  then
    echo "Select the NUMBER below: "
    select ns in "/robot1" "/robot2" "/robot3" "other"; do
      case $ns in
        /robot1) export ROBOT_NS=$ns;break;;
        /robot2) export ROBOT_NS=$ns;break;;
        /robot3) export ROBOT_NS=$ns;break;;
        other)  read -p "Enter the NAMESPACE below:" ns
                export ROBOT_NS=$ns;break;;
      esac
    done
    echo "\$ROBOT_NS was already setting by '$ROBOT_NS'"
    Check
  elif [ $1 == "CHANGE_CAMERA_SERIAL" ]
  then
    read -p "Enter the Serial Number: " sn
    export CAMERA_SERIAL=$sn
    echo "\$CAMERA_SERIAL was already setting by '$CAMERA_SERIAL'"
    Check
  fi
  echo -e "export ROBOT_NS=$ROBOT_NS\nexport CAMERA_SERIAL=$CAMERA_SERIAL" > $SCRIPT_PATH/.robot_setting
}

if [ -z "$ROBOT_NS" ]
then
  echo "\$ROBOT_NS is empty."
  Change "CHANGE_ROBOT_NUMBER"
else
  echo "\$ROBOT_NS is '$ROBOT_NS' now."
  read -t 10 -p "Do you want to change? (y/n)" response
  case $response in
    [yY][eE][sS] | [yY])
      Change "CHANGE_ROBOT_NUMBER"
      ;;
    [nN][oO] | [nN])
      Check
      echo "See you, bye."
      ;;
    *)
      echo "Error. Please try again."
      exit 1
  esac
fi

if [ -z "$CAMERA_SERIAL" ]
then
  echo "\$CAMERA_SERIAL is empty."
  Change "CHANGE_CAMERA_SERIAL"
else
  echo "\$CAMERA_SERIAL is '$CAMERA_SERIAL' now."
  read -t 10 -p "Do you want to change? (y/n)" response
  case $response in
    [yY][eE][sS] | [yY])
      Change "CHANGE_CAMERA_SERIAL"
      ;;
    [nN][oO] | [nN])
      Check
      echo "See you, bye."
      ;;
    *)
      echo "Error. Please try again."
      exit 1
  esac
fi