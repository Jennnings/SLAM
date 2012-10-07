/* 
 * File:   DisplayScreen.cpp
 * Author: bhuvnesh
 * 
 * Created on 7 October, 2012, 3:05 AM
 */

#include "DisplayScreen.h"

DisplayScreen::DisplayScreen(){

    //  Initializing display variables
    win3D.Create("3D Map Window", 1000, 1000);
    win3D.setCameraAzimuthDeg(140);
    win3D.setCameraElevationDeg(20);
    win3D.setCameraZoom(1.0);
    win3D.setFOV(240);
    win3D.setCameraPointingToPoint(2.5, 0, 0);

    gl_points = mrpt::opengl::CPointCloudColoured::Create();
    gl_points->setPointSize(5);

    mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
    // Create the Opengl object for the point cloud:
    scene->insert(gl_points);
    scene->insert(mrpt::opengl::CGridPlaneXY::Create(-500, 500, -500, 500, 0, 100));
    scene->insert(mrpt::opengl::stock_objects::RobotPioneer());
    scene->insert(mrpt::opengl::stock_objects::CornerXYZ());
    win3D.unlockAccess3DScene();
    win3D.repaint();

}

void DisplayScreen::displayScanData(CObservation3DRangeScanPtr scan_data)
{
    CColouredPointsMap temp_map;
    temp_map.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
    temp_map.loadFromRangeScan(*scan_data);
    win3D.get3DSceneAndLock();
    gl_points->loadFromPointsMap(&temp_map);
    win3D.unlockAccess3DScene();
    win3D.repaint();
//    temp_map.~CColouredPointsMap();
}

DisplayScreen::~DisplayScreen() {
}

