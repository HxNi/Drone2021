#!/bin/sh
if [ $# -eq 1 ]
then
	if [ $1 = "ue" ]
	then killall UE4Editor; killall UE4Editor
	else
	echo "OPTION: ue\r"
	fi
else
	killall -9 depth_noise.sh
	killall -9 mavros.sh
	killall -9 px4.sh
	killall -9 airsim.sh
fi
