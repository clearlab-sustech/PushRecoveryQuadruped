#!/bin/bash

make -j6

result=1
PROC_NAME="./sim/sim_trotting"
while [ $result -le 10 ]
do
	ProcNumber=`ps -ef |grep -w $PROC_NAME|grep -v grep|wc -l`  
	if [ $ProcNumber -le 0 ];then  
   		result=0
		./sim/sim &
		sleep 2.5
		./user/MIT_Controller/mit_ctrl m s &
		sleep 1
	else  
  		result=1
		ps -ef |grep -w $PROC_NAME|grep -v grep
		kill $(ps aux | grep $PROC_NAME | awk '{print $2}')
	fi  
	echo $ProcNumber
	sleep 60
done
