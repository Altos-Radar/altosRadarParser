# altosRadarRosParser

## Build
1. git clone https://github.com/Altos-Radar/altosRadarParser.git
2. cd altosRadarParser
3. catkin_make

## Set Parameters
V4: altosRadarParser/src/altosparser/param/altosParserV4.yaml  
RCU: altosRadarParser/src/altosparser/param/altosParserRCU.yaml  
set topic name, installation parameters, IP, port, etc.  

## Run
V4: bash startV4.sh  
RCU: bash startRcu.sh  
V4+RCU: bash startBoth.sh