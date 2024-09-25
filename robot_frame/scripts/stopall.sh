#!/bin/bash
PPIDx=$( ps -a -o pid,ppid,cmd | grep -E "humble|gzserver|gzclient" | grep -v "grep" | awk '{print $1}' ) 
if [[ $PPIDx != "" ]]; then kill -KILL ${PPIDx}; fi
