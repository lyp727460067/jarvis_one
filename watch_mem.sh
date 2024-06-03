#!/bin/bash
pid=$1  #获取进程pid
echo $pid
interval=1  #设置采集间隔
while true
do
    # echo $(date +"%y-%m-%d %H:%M:%S")
    free -h |grep Mem | awk -F ' ' ' {print $3}'
    cat  /proc/$pid/status|grep -e VmRSS
    cpu=`top -n 1 -p $pid|tail -3|head -1|awk '{ssd=NF-4} {print $ssd}'`    #获取cpu占用
    echo "Cpu:% " $cpu
    echo $blank
    sleep $interval
done