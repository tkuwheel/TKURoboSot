#!/bin/bash
# Program:
#     This program will setting environment variable for ros namespace.
# History:
# 2019/3/19 NEET First Edit

SCRIPT_PATH=`cd "$( dirname ${BASH_SOURCE[0]})" && pwd`

if [ "${BASH_SOURCE[0]}" -ef "$0" ]
then
    echo "Hey, you should source this script, not execute it!"
    exit 1
fi

function Check() {
  if grep -q "source $SCRIPT_PATH/.robot_ns" "$HOME/.bashrc";
  then
    echo "OK"
  else
    echo "source $SCRIPT_PATH/.robot_ns >> ~/.bashrc"
    echo "source $SCRIPT_PATH/.robot_ns" >> ~/.bashrc
  fi
}

function Change() {
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
  echo "export ROBOT_NS=$ROBOT_NS" > $SCRIPT_PATH/.robot_ns
  Check
}

if [ -z "$ROBOT_NS" ]
then
  echo "\$ROBOT_NS is empty."
  Change
else
  echo "\$ROBOT_NS is '$ROBOT_NS' now."
  read -t 10 -p "Do you want to change? (y/n)" response
  case $response in
    [yY][eE][sS] | [yY])
      Change
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