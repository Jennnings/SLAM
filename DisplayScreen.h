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
private:
    mrpt::gui::CDisplayWindow3D win3D;
    mrpt::opengl::CPointCloudColouredPtr gl_points;
};

#endif	/* DISPLAYSCREEN_H */

