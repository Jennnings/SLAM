/* 
 * File:   main.cpp
 * Author: bhuvnesh
 *
 * Created on 5 October, 2012, 6:04 PM
 */

#include <cstdlib>
#include "KinectInput.h"
#include "DisplayScreen.h"

using namespace std;

/*
 * 
 */
int main(int argc, char** argv)
{
    KinectInput kinect;
    DisplayScreen display;
    int count = 0;
    while(1)
    {
        kinect.grabKinectFrame();
        CObservation3DRangeScanPtr kinect_scan = kinect.get3DRangeScanData();
        CObservationIMUPtr kinect_imu_data = kinect.getIMUData();
        printf("%d. ",count++);
        if(kinect_scan && kinect_scan->timestamp != 0)
        {
            display.displayScanData(kinect_scan);           
        }
        if(kinect_imu_data && kinect_imu_data->dataIsPresent[IMU_X_ACC])
        {
            printf("imu: %.02f", kinect_imu_data->rawMeasurements[IMU_X_ACC]);
        }
        printf("\n");
        
    }
    return 0;
}

