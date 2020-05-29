source devel_isolated/r200_publisher/setup.bash
while :
do
  rosrun r200_publisher r200_publisher
  sudo modprobe -r uvcvideo
  sudo modprobe uvcvideo
done
