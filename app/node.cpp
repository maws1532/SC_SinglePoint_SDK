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
    uint8_t signal;
    float   distance;
    double curtime;
}RslidarDataComplete;

using namespace std;
using namespace dtfeverest::dtfhwdrivers;
double GetCurTime()
{
    double time = 0;
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast <std::chrono::milliseconds>(duration);
    time = milliseconds.count();
    return time;
}

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
        std::string str1 = robotics_lidar.GetLidarversion();
        printf("Lidar VersionInfo:%s \n", str1.c_str());
        u8 len = strlen(robotics_lidar.GetLidarSNCode());
        char *ch = robotics_lidar.GetLidarSNCode();
        printf("SN:");
        for(int i = 0;i < len;i++)
            printf("%02x", ch[i]);
        printf("\n");
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
                TLidarScan lidar_scan = robotics_lidar.getLidarScan();
                size_t lidar_scan_size = lidar_scan.getSize();
                if(lidar_scan_size > 0)
                {
                    std::vector<RslidarDataComplete> send_lidar_scan_data_part;
                    send_lidar_scan_data_part.resize(lidar_scan_size);
                    RslidarDataComplete one_lidar_data;
                    for(size_t i = 0; i < lidar_scan_size; i++)
                    {
                        one_lidar_data.signal = lidar_scan.signal[i];
                        one_lidar_data.distance = lidar_scan.distance[i];
                        one_lidar_data.curtime = lidar_scan.CurTime[i];
                        send_lidar_scan_data_part[i] = one_lidar_data;
                    }
                    printf("SYtem Current time:%5.2f getUartdatatime:%5.2f count:%3d signal:%d dis:%5.2f\n",GetCurTime(), lidar_scan.CurTime[0], lidar_scan_size,  lidar_scan.signal[0],  lidar_scan.distance[0]);
                    robotics_lidar.m_lidar_scan.clear();
                }
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
        //usleep(50);
    }

    return 0;
}