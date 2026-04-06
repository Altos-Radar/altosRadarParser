source devel/setup.bash
#gnome-terminal --tab -t "rviz1" -e  'bash -c "roslaunch rviz1.launch;read"'
#sleep 2s
#gnome-terminal --tab -t "rviz2" -e  'bash -c "roslaunch rviz2.launch;read"'
sleep 1s
gnome-terminal --tab -t "altosradar" -e  'bash -c "rosrun altosradar altosRadarParse;read"'