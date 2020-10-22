#!/bin/bash

rtSendPeriod=100000
beSendPeriod=100000
rtComputeBusyLoop=25000
beComputeBusyLoop=100000
while [  $rtComputeBusyLoop -le  160000 ]; do
  echo "Rcv RT Compute Period is $rtComputeBusyLoop"
  ros2 run cbg_executor_demo ping-pong io $rtSendPeriod $beSendPeriod $rtComputeBusyLoop $beComputeBusyLoop >> resultsComputationLoad.txt
  let rtComputeBusyLoop=rtComputeBusyLoop+25000
done
