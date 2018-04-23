if [ $# -ne 1 ] ; then
    echo "Please add your own IP as parameter."
else
    export ROS_IP=$1
    export ROS_MASTER_URI=http://192.168.1.31:11311
fi

