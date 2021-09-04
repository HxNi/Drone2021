#!/bin/sh
cd ~
if [ $# -eq 1 ]
then
	if [ $1 = "ue" ]
	then gnome-terminal -- run/ue.sh
	elif [ $1 = "airsim" ]
	then gnome-terminal --geometry=+0+0 -- run/airsim.sh
	elif [ $1 = "px4" ]
	then gnome-terminal --geometry=-0+0 -- run/px4.sh
	elif [ $1 = "mavros" ]
	then gnome-terminal --geometry=+0-0 -- run/mavros.sh
	elif [ $1 = "depth_noise" ]
	then gnome-terminal --geometry=-0-0 -- run/depth_noise.sh
	else
	echo "OPTION: ue airsim px4 mavros depth_noise\r"
	fi
else
	gnome-terminal --geometry=+0+0 -- run/airsim.sh
	sleep 3s
	gnome-terminal --geometry=-0+0 -- run/px4.sh
	sleep 3s
	gnome-terminal --geometry=+0-0 -- run/mavros.sh
	sleep 3s
	gnome-terminal --geometry=-0-0 -- run/depth_noise.sh
fi
