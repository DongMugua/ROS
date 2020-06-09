#! /bin/bash

echo -n "select run mode. 'entity','simulation' e/s:"
read FILM

if [ ${FILM} == 'e' ]; then
    # setup for bodyhub
    sudo chmod 666 /dev/ttyUSB0
    if [ $? -ne 0 ]; then
        echo "Please connect the serial port to the USB port!"
        exit 0
    fi

    setserial /dev/ttyUSB0 low_latency
    if [ $? -ne 0 ]; then
        echo "Set up low_latency failed!"
        exit 0
    fi

    # open another terminal and launch the node
    gnome-terminal -x bash -c "roslaunch bodyhub bodyhub.launch" &
elif [ ${FILM} == 's' ]; then
    # locate vrep.sh and the vrep scene file
    sudo updatedb

    vrep=$(locate vrep.sh)
    scenename=$(locate */bodyhub/vrep/Roban.ttt)

    if [ ! $vrep ]; then
        echo "Could not locate vrep!"
        exit 1
    fi

    if [ ! $scenename ]; then
        echo "Could not locate scene file!"
        exit 2
    fi

    # open another terminal and launch the simulation
    gnome-terminal -x bash -c "roslaunch bodyhub bodyhub.launch sim:=true " &

    # open vrep and load the simulation scene
    $vrep $scenename &
else
    echo "Please select the given option!"
    exit 3
fi

wait
echo "BodyHubNode stopped!"
