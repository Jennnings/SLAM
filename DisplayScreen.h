/* 
 * File:   DisplayScreen.h
 * Author: bhuvnesh
 *
 * Created on 7 October, 2012, 3:05 AM
 */



#ifndef DISPLAYSCREEN_H
#define	DISPLAYSCREEN_H

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

class DisplayScreen {
public:
    DisplayScreen();
    virtual ~DisplayScreen();
    void displayScanData(CObservation3DRangeScanPtr scan_data);
    void displayScanData(CObservation3DRangeScanPtr scan_data, CPose3D curr_pose);
    void displayBotPositions(vector<CPose3D> bot_path);
    void displayBotPositions(CPose3D bot_pose);
    void updateGlobalMap(CObservation3DRangeScanPtr scan_data, CPose3D curr_pose);
    void initializeGlobalMap(CObservation3DRangeScanPtr scan_data);
private:
    mrpt::gui::CDisplayWindow3D win3D;
    mrpt::opengl::CPointCloudColouredPtr gl_points;//for local map
    mrpt::opengl::CPointCloudColouredPtr gl_points_map;//for global map
    mrpt::opengl::CSetOfObjectsPtr gl_cur_cam_corner;
    mrpt::opengl::CSetOfObjectsPtr gl_keyframes; //for camera_key_frame_path
    CColouredPointsMap global_map;
    
};

#endif	/* DISPLAYSCREEN_H */

