/* 
 * File:   KinectInput.h
 * Author: bhuvnesh
 *
 * Created on 5 October, 2012, 6:15 PM
 */

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

#ifndef KINECTINPUT_H
#define	KINECTINPUT_H

class KinectInput {
public:
    KinectInput();
    virtual ~KinectInput();
    CObservation3DRangeScanPtr get3DRangeScanData();
    CObservationIMUPtr getIMUData();
    void grabKinectFrame();
    
private:
    struct KinectParam {
        CObservation3DRangeScanPtr new_obs;
        CObservationIMUPtr new_imu_obs;
    };
    CKinect kinect;
    KinectParam kinect_params;
};

#endif	/* KINECTINPUT_H */

