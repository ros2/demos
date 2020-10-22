#!/bin/bash

rtSendPeriod=4000
beSendPeriod=1000
rtComputeBusyLoop=0
beComputeBusyLoop=1000
while [  $rtSendPeriod -ge 31 ]; do
  echo "rtSendPeriod in us is $rtSendPeriod"
  ros2 run cbg_executor_demo ping-pong io $rtSendPeriod $beSendPeriod $rtComputeBusyLoop $beComputeBusyLoop >> resultsIntraProcessCommunicationLoad.txt
  let rtSendPeriod=rtSendPeriod/2
done
