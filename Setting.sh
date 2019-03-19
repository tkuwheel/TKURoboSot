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
  if grep -q "source $SCRIPT_PATH/.robot" "$HOME/.bashrc";
  then
    echo "OK"
  else
    echo "source $SCRIPT_PATH/.robot >> ~/.bashrc"
    echo "source $SCRIPT_PATH/.robot" >> ~/.bashrc
  fi
}

function Change() {
  echo "Select the NUMBER below: "
  select ns in "/robot1" "/robot2" "/robot3" "other"; do
    case $ns in
      /robot1) export ROBOT=$ns;break;;
      /robot2) export ROBOT=$ns;break;;
      /robot3) export ROBOT=$ns;break;;
      other)  read -p "Enter the NAMESPACE below:" ns
              export ROBOT=$ns;break;;
    esac
  done
  echo "\$ROBOT was already setting by '$ROBOT'"
  echo "export ROBOT=$ROBOT" > $SCRIPT_PATH/.robot
  Check
}

if [ -z "$ROBOT" ]
then
  echo "\$ROBOT is empty."
  Change
else
  echo "\$ROBOT is '$ROBOT' now."
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