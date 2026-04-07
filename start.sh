source devel/setup.bash
gnome-terminal --tab -t "rviz" -e  'bash -c "roslaunch rviz.launch;read"'
sleep 2s
rosparam load ./src/altosrcu/param/altosRcuParameters.yaml
gnome-terminal --tab -t "altosRadarRcu" -e  'bash -c "rosrun altosrcu altosrcu;read"'