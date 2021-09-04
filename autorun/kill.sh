#!/bin/sh
if [ $# -eq 1 ]
then
	if [ $1 = "ue" ]
	then killall UE4Editor; killall UE4Editor
	else
	echo "OPTION: ue\r"
	fi
else
	killall depth_noise.sh
	killall mavros.sh
	killall px4.sh
	killall airsim.sh
fi
