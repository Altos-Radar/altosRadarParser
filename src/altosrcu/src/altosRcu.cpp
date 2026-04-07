#include "pointCloud.h"
#include <algorithm>
#include <arpa/inet.h>
#include <errno.h>
#include <iostream>
#include <math.h>
#include <netinet/in.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <array>
using namespace std;
#define widthSet 8000
#define vrMax 60
#define vrMin -60
#define vStep 0.1
#define errThr 3
#define PI 3.1415926
#define GROUPIP "224.1.2.4"
#define GROUPPORT 4040
#define LOCALIP "192.168.3.1"
#define UNIPORT 4041
#define UNIFLAG 0
#define INSTALLHEIGHT 1.85
#define BASEFRAMEID "base"

float rcsCal(float range, float azi, float snr, float* rcsBuf) {
    int ind = (azi * 180 / PI + 60.1) * 10;
    float rcs = powf32(range, 2.6) * snr / 5.0e6 / rcsBuf[ind];

    return rcs;
}

int socketGen()
{
    struct sockaddr_in addr;

    struct ip_mreq req;
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (-1 == sockfd) {
        perror("socket");
        return 0;
    }
    struct timeval timeout = {1, 300};
    setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout,
               sizeof(struct timeval));
    memset(&addr, 0, sizeof(addr));
    if(UNIFLAG)
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(UNIPORT);
        addr.sin_addr.s_addr = inet_addr(LOCALIP);
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return 0;
        }
    }else
    {
        addr.sin_family = AF_INET;
        addr.sin_port = htons(GROUPPORT);
        addr.sin_addr.s_addr = INADDR_ANY;
        int ret = bind(sockfd, (struct sockaddr*)&addr, sizeof(addr));
        if (-1 == ret) {
            perror("bind");
            return 0;
        }

        req.imr_multiaddr.s_addr = inet_addr(GROUPIP);
        req.imr_interface.s_addr = inet_addr(LOCALIP);
        ;
        ret = setsockopt(sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req));
        if (ret < 0) {
            perror("setsockopt");
            return 0;
        }
    }

    return sockfd;
}

float hist(vector<POINTCLOUD> pointCloudVec, float* histBuf, float step) {
    int ind = 0;
    float vr = 0;

    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < 30; j++) {
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                vr = pointCloudVec[i].point[j].doppler /
                     cos(pointCloudVec[i].point[j].azi);
                ind = (vr - vrMin) / step;
                if (vr > 60 || vr < -60 || isnan(vr)) {
                    continue;
                }
                if (vr <= 0) {
                    histBuf[ind]++;
                }
            }
        }
    }
    return float(
               (max_element(histBuf, histBuf + (int((vrMax - vrMin) / step))) -
                histBuf)) *
               step +
           vrMin;
}

void calPoint(vector<POINTCLOUD> pointCloudVec,pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud,int installFlag,float *rcsBuf,float step,float *histBuf,int pointNumPerPack)
{
    pcl::PointXYZHSV cloudPoint;
    for(int i = 0;i<pointCloudVec.size();i++)
    {
        for(int j = 0;j<pointNumPerPack;j++)
        {
            if(abs(pointCloudVec[i].point[j].range)>0&&abs(pointCloudVec[i].point[j].azi)<=80*PI/180)
            {
                pointCloudVec[i].point[j].ele = installFlag*(pointCloudVec[i].point[j].ele+0*PI/180);

                cloudPoint.x = (pointCloudVec[i].point[j].range)*cos(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele); 
                cloudPoint.y = (pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].azi)*cos(pointCloudVec[i].point[j].ele);; 
                cloudPoint.z = (pointCloudVec[i].point[j].range)*sin(pointCloudVec[i].point[j].ele) ; 
                if(cloudPoint.z < -INSTALLHEIGHT)
                {
                    cloudPoint.z = -cloudPoint.z - 2*INSTALLHEIGHT;
                }
                cloudPoint.h = pointCloudVec[i].point[j].doppler; 
                cloudPoint.s = pointCloudVec[i].point[j].snr;//rcsCal(pointCloudVec[i].point[j].range,pointCloudVec[i].point[j].azi,pointCloudVec[i].point[j].snr,rcsBuf);
                cloud->push_back(cloudPoint);
            }
        }
    }
    memset(histBuf, 0, sizeof(float) * int((vrMax - vrMin) / step));
    float vrEst = hist(pointCloudVec, histBuf, step);
    float tmp;
    for (int i = 0; i < pointCloudVec.size(); i++) {
        for (int j = 0; j < pointNumPerPack; j++) {
            if(i*pointNumPerPack+j>=cloud->size())
            {
                break;
            }
            if (abs(pointCloudVec[i].point[j].range) > 0) {
                tmp = (cloud->points[i * pointNumPerPack + j].h) -
                      vrEst * cos(pointCloudVec[i].point[j].azi);
                if (tmp < -errThr) {
                    cloud->points[i * pointNumPerPack + j].v = -1;
                } else if (tmp > errThr) {
                    cloud->points[i * pointNumPerPack + j].v = 1;
                } else {
                    cloud->points[i * pointNumPerPack + j].v = 0;
                }
            }
        }
    }
}

int main(int argc, char** argv) {
    
    // rcs read
    float* rcsBuf = (float*)malloc(1201 * sizeof(float));
    FILE* fp_rcs = fopen("data//rcs.dat", "rb");
    if (fp_rcs == NULL)
    {
        printf("[WARNING] data/rcs.dat not found in pwd [WARNING]\n");
        return 0;
    } 
    fread(rcsBuf, 1201, sizeof(float), fp_rcs);
    fclose(fp_rcs);

    // ros Init
    ros::init(argc, argv, "altosRcu");
    ros::NodeHandle nh;

    // get param
    std::string topicName[4]={"","","",""};
    std::array<std::array<double, 6>, 3> installationParam;
    nh.getParam("altosRcuParameters/radar0/topicName", topicName[0]);
    nh.getParam("altosRcuParameters/radar1/topicName", topicName[1]);
    nh.getParam("altosRcuParameters/radar2/topicName", topicName[2]);
    nh.getParam("altosRcuParameters/radar3/topicName", topicName[3]);

    std::vector<double> tempVec;
    nh.getParam("altosRcuParameters/radar0/installationParam", tempVec);
    std::copy(tempVec.begin(), tempVec.end(), installationParam[0].begin());
    nh.getParam("altosRcuParameters/radar1/installationParam", tempVec);
    std::copy(tempVec.begin(), tempVec.end(), installationParam[1].begin());
    nh.getParam("altosRcuParameters/radar2/installationParam", tempVec);
    std::copy(tempVec.begin(), tempVec.end(), installationParam[2].begin());
    nh.getParam("altosRcuParameters/radar3/installationParam", tempVec);
    std::copy(tempVec.begin(), tempVec.end(), installationParam[3].begin());

    //pub
    ros::Publisher pubArray[4];
    pubArray[0]=nh.advertise<sensor_msgs::PointCloud2>(topicName[0], 1);
    pubArray[1]=nh.advertise<sensor_msgs::PointCloud2>(topicName[1], 1);
    pubArray[2]=nh.advertise<sensor_msgs::PointCloud2>(topicName[2], 1);
    pubArray[3]=nh.advertise<sensor_msgs::PointCloud2>(topicName[3], 1);
    ros::Publisher markerPub = nh.advertise<visualization_msgs::Marker>("TEXT_VIEW_FACING", 10);
    ros::Publisher originPub = nh.advertise<visualization_msgs::Marker>("origin", 10);

    //tf
    static tf::TransformBroadcaster tfBr;
    tf::Transform transform;
    tf::Quaternion q;

    sensor_msgs::PointCloud2 output;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZHSV>);

    visualization_msgs::Marker origin;
    origin.header.frame_id = BASEFRAMEID;
    origin.type = visualization_msgs::Marker::SPHERE;
    origin.action = visualization_msgs::Marker::ADD;

    origin.pose.position.x = 0;
    origin.pose.position.y = 0;
    origin.pose.position.z = 0;
    origin.pose.orientation.x = 0;
    origin.pose.orientation.y = 0;
    origin.pose.orientation.z = 0;
    origin.pose.orientation.w = 1;

    origin.scale.x = 3;
    origin.scale.y = 3;
    origin.scale.z = 3;
    origin.color.r = 1.0;
    origin.color.g = 1.0;
    origin.color.b = 0.0;
    origin.color.a = 1;

    visualization_msgs::Marker marker;
    marker.ns = "basic_shapes";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.id =0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 10;
    marker.color.b = 1.0f;
    marker.color.g = 1.0f;
    marker.color.r = 1.0f;
    marker.color.a = 1;
    geometry_msgs::Pose pose;
    pose.position.x =  (float)-5;
    pose.position.y =  0;
    pose.position.z =0;
    
    // socket Gen
    struct sockaddr_in  from;
    socklen_t           len = sizeof(from);
    int                 sockfd = socketGen();
    
    // pointcloud recv para
    vector<POINTCLOUD>  pointCloudVec0, pointCloudVec1, pointCloudVec2, pointCloudVec3;
    POINTCLOUD          pointCloudBuf;
    char*               recvBuf = (char*)&pointCloudBuf;
    int                 installFlag = -1;
    int                 pointNumPerPack = 30;
    int                 pointSizeByte = 44;
    int                 recvFrameLen = 0;
    int                 frameNum = 0;
    int                 frameId[2] = {0, 0};
    int                 cntPointCloud[4] = {0, 0, 0, 0};
    float               vrEst = 0;
    unsigned short      curObjInd[4] = {0, 0, 0, 0};
    unsigned int        radarId;
    unsigned char       mode;
    float*              histBuf = (float*)malloc(sizeof(float) * int((vrMax - vrMin) / vStep));

    while(ros::ok())
    {
        memset(recvBuf,0,sizeof(POINTCLOUD));
        // printf("%s",topicName[0].c_str()); //debug
        int ret = recvfrom(sockfd, recvBuf, sizeof(POINTCLOUD), 0, (struct sockaddr *)&from, &len);
        if (ret > 0)
		{
            radarId = pointCloudBuf.pckHeader.reserved[0]+1;
            curObjInd[radarId] = pointCloudBuf.pckHeader.curObjInd;
            // mode = pointCloudBuf.pckHeader.mode;
            cntPointCloud[radarId] = pointCloudBuf.pckHeader.objectCount;
            switch (radarId)
            {
                case 0: pointCloudVec0.push_back(pointCloudBuf);break;
                case 1: pointCloudVec1.push_back(pointCloudBuf);break;
                case 2: pointCloudVec2.push_back(pointCloudBuf);break;
                case 3: pointCloudVec3.push_back(pointCloudBuf);break;
                default:
                    ROS_ERROR("wrong radarId = %u in PCKHEADER.reserved", radarId);
                    continue; 
                    
            }
            
            if (((curObjInd[radarId] + 1) * pointNumPerPack >= pointCloudBuf.pckHeader.objectCount)) {
                // if (pointCloudVec.size() * pointNumPerPack < cntPointCloud[0] + cntPointCloud[1]) {
                //     printf(
                //         "FrameId %d %ld Loss %ld pack(s) in %d "
                //         "packs------------------------\n",
                //         pointCloudBuf.pckHeader.frameId, pointCloudVec.size(),
                //         int(ceil(cntPointCloud[0] / pointNumPerPack) +
                //             ceil(cntPointCloud[1] / pointNumPerPack)) -
                //             pointCloudVec.size(),
                //         int(ceil(cntPointCloud[0] / pointNumPerPack) +
                //             ceil(cntPointCloud[1] / pointNumPerPack)));
                // }
                switch (radarId)
                {
                    case 0: calPoint(pointCloudVec0, cloud, installFlag, rcsBuf, vStep,
                         histBuf,pointNumPerPack);
                         break;
                    case 1: calPoint(pointCloudVec1, cloud, installFlag, rcsBuf, vStep,
                         histBuf,pointNumPerPack);
                    case 2: calPoint(pointCloudVec2, cloud, installFlag, rcsBuf, vStep,
                         histBuf,pointNumPerPack);
                    case 3: calPoint(pointCloudVec3, cloud, installFlag, rcsBuf, vStep,
                         histBuf,pointNumPerPack);
                }

                //tf
                transform.setOrigin(tf::Vector3(installationParam[radarId][0], installationParam[radarId][1], installationParam[radarId][2])); // x, y, z
                q.setRPY(installationParam[radarId][3], installationParam[radarId][4], installationParam[radarId][5]); // roll, pitch, yaw
                transform.setRotation(q);
                tfBr.sendTransform(
                    tf::StampedTransform(
                        transform,
                        ros::Time::now(),
                        BASEFRAMEID,
                        topicName[radarId]
                        )
                    );

                //pub point cloud
                pcl::toROSMsg(*cloud, output);
                output.header.frame_id = topicName[radarId];
                output.header.stamp = ros::Time::now();; 
                printf("pointNum of %d frame of %s: %d\n",
                       pointCloudBuf.pckHeader.frameId,
                       topicName[radarId].c_str(),
                       cntPointCloud[radarId]);
                output.header.stamp = ros::Time::now();
                pubArray[radarId].publish(output);

                originPub.publish(origin);

                marker.header.frame_id=topicName[radarId];
                marker.header.stamp = ros::Time::now();
                ostringstream str;
                str<<topicName[radarId]<<" pointNum: "<<cntPointCloud[0];
                marker.text=str.str();
                marker.pose=pose;
                markerPub.publish(marker);

                // clear
                switch (radarId)
                {
                    case 0: pointCloudVec0.clear(); break;
                    case 1: pointCloudVec1.clear(); break;
                    case 2: pointCloudVec2.clear(); break;
                    case 3: pointCloudVec3.clear(); break;
                }
                
                cloud.reset(new pcl::PointCloud<pcl::PointXYZHSV>);
                cntPointCloud[radarId] = 0;
            }
        } else {
            // printf("recv failed (timeOut)   %d\n", ret);
        }
    }

    close(sockfd);
    free(histBuf);
    return 0;
}
