/*
*  3iRoboticsLIDAR System II
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-09-15
*
*/


#include "C3iroboticsLidar.h"
#include "CSerialConnection.h"
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <iostream>
#include <chrono>


#define DEBUG_MAIN

#define DEG2RAD(x) ((x)*M_PI/180.)

typedef struct _rslidar_data
{
    _rslidar_data()
    {
        signal = 0;
        distance = 0.0;
        curtime = 0.0;
    }
    uint8_t ConfidenceDegr;
    uint8_t signal;
    float   distance;
    double curtime;
}RslidarDataComplete;

using namespace std;
using namespace dtfeverest::dtfhwdrivers;
int main(int argc, char * argv[])
{
    int count = 0;
	int    opt_com_baudrate = 115200;//;
    string opt_com_path = "/dev/ttyUSB0";

    CSerialConnection serial_connect;
    C3iroboticsLidar robotics_lidar;
    CLidarUnpacket unpacket;
    serial_connect.setBaud(opt_com_baudrate);
    serial_connect.setPort(opt_com_path.c_str());
    if(serial_connect.openSimple())
    {
        printf("[AuxCtrl] Open serail port sucessful!\n");
        printf("baud rate:%d\n",serial_connect.getBaud());
    }
    else
    {
        printf("[AuxCtrl] Open serail port %s failed! \n", opt_com_path.c_str());
        return -1;
    }
    printf("C3iroboticslidar connected\n");
    robotics_lidar.initilize(&serial_connect);
    
    std::string SDKV = robotics_lidar.GetSDKVersion();
    printf("SDK Version:%s \n",SDKV.c_str());
    if(robotics_lidar.GetDeviceInfo())
    {
        u8 len = robotics_lidar.GetSNCodelen();
        u8 *ch = robotics_lidar.GetLidarSNCode();
        printf("current Systemtime:%5.2f\n",unpacket.GetSystemTimeInSeconds());

        printf("length:%d\n", len);
        printf("SN:");
        for(int i = 0;i < len;i++)
            printf("%02x", ch[i]);
        printf("\n");

        printf("Lidar VersionInfo:%s \n", robotics_lidar.GetLidarversion().c_str());

        printf("lidar info:%s\n", robotics_lidar.GetLidarType().c_str());

        printf("HardwareVersion:%s\n", robotics_lidar.GetLidarHardwareVersion().c_str());

        printf("SoftwareVersion:%s\n", robotics_lidar.GetLidarSoftwareVersion().c_str());

    }
    else
    {
        printf("get Lidar info Fail\n");
    }
    
    while(1)
    {
        //usleep(100000);
		TLidarGrabResult result = robotics_lidar.getScanData();
        switch(result)
        {
            case LIDAR_GRAB_ING:
            case LIDAR_GRAB_SUCESS:
            {
                RslidarDataComplete Lidardata;
                LidarCurrData data = robotics_lidar.getLidardata();
                Lidardata.distance = data.dis;
                Lidardata.signal = data.sig;
                Lidardata.ConfidenceDegr = data.ConfidenceDegr;
                Lidardata.curtime = data.CTime;

                printf("SYtem Current time:%5.2f getUartdatatime:%5.2f signal:%d dis:%5.2f, ConfidenceDegr:%d \n",unpacket.GetSystemTimeInSeconds(), Lidardata.curtime,  Lidardata.signal,  Lidardata.distance, Lidardata.ConfidenceDegr);
                break;
            }
            
            case LIDAR_GRAB_ERRO:
            {
                
               //printf("[Main] Lidar error code = %d \n", robotics_lidar.getLidarError());
               break;
            }
            case LIDAR_GRAB_ELSE:
            {
                printf("[Main] LIDAR_GRAB_ELSE!\n");
                break;
            }
        }
        //usleep(2000);
    }
    serial_connect.closeSerial();
    return 0;
}