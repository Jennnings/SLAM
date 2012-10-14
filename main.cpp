/* 
 * File:   main.cpp
 * Author: bhuvnesh
 *
 * Created on 5 October, 2012, 6:04 PM
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include "KinectInput.h"
#include "DisplayScreen.h"
#include "Tracker.h"
#define DROP_COUNT 10

using namespace std;
//using namespace std::ofstream;
//------------------------------------------------------------------------------
/* 
 * File:   main.cpp
 * Author: bhuvnesh
 *
 * Created on 7 September, 2012, 6:19 PM
 */
/*
   This is a very *simple* approach to SLAM. It would be better to first select
   3d points in the 3D point-cloud, then project them into the image plane
  and then track them (instead of directly choosing poins in the image plane),
          since in that case the (x,y) to 3D correspondence would be much more accurate.
          Feel free to modify or improve this example as you need!

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
using namespace std;


//------------------------------------------------------------------------------
/*
 * 
 */
int main() {
    KinectInput kinect;
    DisplayScreen display;
    Tracker tracker;
    int loop_count = 0;
    CObservation3DRangeScanPtr kinect_scan;
    CObservationIMUPtr kinect_imu_data;

    //dropping first few frames from kinect
    while(loop_count < DROP_COUNT)
    {
        kinect.grabKinectFrame();
        loop_count++;
    }
    loop_count = 0;
    
    // Need to update gl_keyframes from camera_key_frames_path??
    bool gl_keyframes_must_refresh = true; 
    
    //start of actual process.
    while (1) {
        kinect.grabKinectFrame();
        kinect_scan = kinect.get3DRangeScanData();
        kinect_imu_data = kinect.getIMUData();
        
        //data validation checks
        if (!kinect_scan &&!kinect_scan->hasIntensityImage &&
                kinect_scan->timestamp == 0 &&
                size_t(kinect_scan->rangeImage.cols() * kinect_scan->rangeImage.rows()) == kinect_scan->points3D_x.size()) {
            continue;
        }        
        
        if (loop_count > 1) {
            tracker.TrackFeatures(kinect_scan);
        }
        tracker.curVisibleFeats.clear();
        tracker.calculate3DFeatures(kinect_scan);
        
        //load local points map from kinect_scan
        
        if(tracker.calculatePoseMatrix())
        {
            gl_keyframes_must_refresh = true;
            display.updateGlobalMap(kinect_scan,tracker.getCurrentGlobalPose());
        }

        if (tracker.camera_key_frames_path.empty() || tracker.lastVisibleFeats.empty()) {
            // First iteration of process
            tracker.camera_key_frames_path.clear();
            tracker.camera_key_frames_path.push_back(TPose3D(0, 0, 0, 0, 0, 0));
            tracker.lastVisibleFeats = tracker.curVisibleFeats;
            display.initializeGlobalMap(kinect_scan);
        }
        loop_count++;
        //updating the previous image
        tracker.previous_image = kinect_scan->intensityImage;
        
        //displaying the 3D scan of kinect.
        display.displayScanData(kinect_scan, tracker.getCurrentGlobalPose());
        display.displayBotPositions(tracker.getCurrentGlobalPose());
//        
//        //printing some information for debugging
//        cout <<  loop_count <<":" << tracker.getCurrentGlobalPose().asString() << endl;
    }
    return 0;
}
