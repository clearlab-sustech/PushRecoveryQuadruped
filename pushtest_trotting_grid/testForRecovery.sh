#!/bin/bash

make -j6

result=1
PROC_NAME="./sim/sim_trotting"
while [ $result -le 10 ]
do
	ProcNumber=`ps -ef |grep -w $PROC_NAME|grep -v grep|wc -l`  
	if [ $ProcNumber -le 0 ];then  
   		result=0
		./sim/sim_trotting &
		sleep 2.5
		./user/MIT_Controller/mit_ctrl_trotting m s &
		sleep 15
	else  
  		result=1
		ps -ef |grep -w $PROC_NAME|grep -v grep
		kill $(ps aux | grep $PROC_NAME | awk '{print $2}')
	fi  
	echo $ProcNumber
	sleep 35
done
