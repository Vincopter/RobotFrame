#!/bin/bash
PPIDx=$( ps -a -o pid,ppid,cmd | grep "humble" | grep -v "grep" | awk '{print $1}' ) 
if [[ $PPIDx != "" ]]; then kill -KILL ${PPIDx}; fi
