/* 
 * File:   Tracker.h
 * Author: bhuvnesh
 *
 * Created on 9 October, 2012, 8:09 PM
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

#ifndef TRACKER_H
#define	TRACKER_H

class Tracker {
public:
    Tracker();
    virtual ~Tracker();
    void TrackFeatures(CObservation3DRangeScanPtr scan);
    void calculate3DFeatures(CObservation3DRangeScanPtr scan);
    bool calculatePoseMatrix();
    CPose3D getCurrentGlobalPose();
    CImage previous_image;
    map<TFeatureID, TPoint3D> curVisibleFeats;
    map<TFeatureID, TPoint3D> lastVisibleFeats;
    std::vector<TPose3D> camera_key_frames_path;
private:
    CGenericFeatureTrackerAutoPtr  tracker;
    CFeatureList tracked_features;
    CPose3D currentCamPose_wrt_last; // wrt last pose in "camera_key_frames_path"
    
};

#endif	/* TRACKER_H */

