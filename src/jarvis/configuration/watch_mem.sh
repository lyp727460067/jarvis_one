#!/bin/bash
pid=`top -n 1 |grep mpslam |head -1 |awk '{ssd=NF-7} {print $ssd}'`
# pid=$1  #获取进程pid
echo $pid
interval=100  #设置采集间隔
echo $(date +"%y-%m-%d %H:%M:%S")
while true
do
    echo $(date +"%y-%m-%d %H:%M:%S")
    free -h |grep Mem | awk -F ' ' ' {print $3}'
    cat  /proc/$pid/status|grep -e VmRSS
    cat  /proc/$pid/status|grep -e VmRSS >>proc_memlog.txt
    top -n 1 |grep mpslam |head -1 
    top -n 1 |grep  sensor_node |head -1 
    top -n 1 |grep mpslam |head -1 >> proc_memlog.txt
    top -n 1 |grep sensor_node|head -1 >> proc_memlog.txt
    echo $blank
    usleep $interval
done
echo $(date +"%y-%m-%d %H:%M:%S")