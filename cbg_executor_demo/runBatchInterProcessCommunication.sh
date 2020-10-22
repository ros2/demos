#!/bin/bash

rtSendPeriod=4000
beSendPeriod=1000
rtComputeBusyLoop=0
beComputeBusyLoop=1000
while [  $rtSendPeriod -ge 31 ]; do
  echo "rtSendPeriod in us is $rtSendPeriod"
  ros2 run cbg_executor_demo ping-pong o $rtSendPeriod $beSendPeriod $rtComputeBusyLoop $beComputeBusyLoop 0 >> resultsInterProcessCommunicationLoad.txt &
  receiverPid=$!
  sleep 2
  ros2 run cbg_executor_demo ping-pong i $rtSendPeriod $beSendPeriod $rtComputeBusyLoop $beComputeBusyLoop 0 >> resultsInterProcessCommunicationLoad.txt
  wait $receiverPid
  let rtSendPeriod=rtSendPeriod/2
done
