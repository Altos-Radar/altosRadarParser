source devel/setup.bash
gnome-terminal --tab -t "rviz" -e 'bash -c "roslaunch rviz.launch;read"'
sleep 2s
gnome-terminal --tab -t "altosParserV4" -e 'bash -c "roslaunch altosparser altosParserV4.launch;read"'
sleep 2s
gnome-terminal --tab -t "altosParserRcu" -e 'bash -c "roslaunch altosparser altosParserRcu.launch;read"'