/* 
 * File:   KinectInput.cpp
 * Author: bhuvnesh
 * 
 * Created on 5 October, 2012, 6:15 PM
 */

#include "KinectInput.h"
#include "mrpt/hwdrivers.h"
#include "mrpt/gui.h"
#include "mrpt/maps.h"
#include "mrpt/vision.h"
#include "mrpt/scanmatching.h"
#include "mrpt/system/filesystem.h"

using namespace mrpt;
using namespace mrpt::vision;
using namespace mrpt::hwdrivers;
using namespace mrpt::poses;
using namespace mrpt::math;
using namespace mrpt::gui;
using namespace mrpt::utils;

void KinectInput::grabKinectFrame() {
    bool there_is_obs = true;
    bool hw_error = false;
    try {
        do{
            CObservation3DRangeScanPtr obs = CObservation3DRangeScan::Create();
            CObservationIMUPtr imu_obs = CObservationIMU::Create();
            kinect.getNextObservation(*obs, *imu_obs, there_is_obs, hw_error);
            if (there_is_obs && !hw_error) {
                kinect_params.new_obs = obs;
                kinect_params.new_imu_obs = imu_obs;
            }
        }while (!there_is_obs || hw_error);
    } catch (exception ex) {
        printf("Capturing Exception: %s", ex.what());
    }
}

CObservation3DRangeScanPtr KinectInput::get3DRangeScanData() {
    return kinect_params.new_obs;
}

CObservationIMUPtr KinectInput::getIMUData() {
    return kinect_params.new_imu_obs;
}

KinectInput::KinectInput() {
    try {
        kinect.enableGrab3DPoints(true);
        kinect.enableGrabAccelerometers(true);
        kinect.enableGrabDepth(true);
        kinect.enableGrabRGB(true);
        kinect.initialize();
    } catch (exception ex) {
        printf("Initialization Exception: %s", ex.what());
    }
}

KinectInput::~KinectInput() {
    kinect.~CKinect();
}

