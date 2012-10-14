/* 
 * File:   DisplayScreen.cpp
 * Author: bhuvnesh
 * 
 * Created on 7 October, 2012, 3:05 AM
 */

#include "DisplayScreen.h"

DisplayScreen::DisplayScreen(){
    
    global_map.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
    
    //  Initializing display variables
    win3D.Create("Kinect VSLAM", 800, 600);
    win3D.setCameraAzimuthDeg(140);
    win3D.setCameraElevationDeg(20);
    win3D.setCameraZoom(8.0);
    win3D.setFOV(90);
    win3D.setCameraPointingToPoint(2.5, 0, 0);
    
    
    gl_points = mrpt::opengl::CPointCloudColoured::Create();
    gl_points->setPointSize(2.5);
    
    gl_points_map = mrpt::opengl::CPointCloudColoured::Create();
    gl_points_map->setPointSize(5);
    
    gl_cur_cam_corner = mrpt::opengl::stock_objects::CornerXYZSimple(0.4, 4);
    gl_keyframes = mrpt::opengl::CSetOfObjects::Create();
    
    mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();
    scene->insert(gl_points);
    scene->insert(gl_points_map);
    scene->insert(gl_keyframes);
    scene->insert(mrpt::opengl::CGridPlaneXY::Create(-500, 500, -500, 500, 0, 100));
    scene->insert(gl_cur_cam_corner);
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
}

void DisplayScreen::displayScanData(CObservation3DRangeScanPtr scan_data, CPose3D curr_pose)
{
    CColouredPointsMap temp_map;
    temp_map.colorScheme.scheme = CColouredPointsMap::cmFromIntensityImage;
    temp_map.loadFromRangeScan(*scan_data);
    win3D.get3DSceneAndLock();
    gl_points->loadFromPointsMap(&temp_map);
    gl_points->setPose(curr_pose);
    gl_cur_cam_corner->setPose(curr_pose);
    win3D.unlockAccess3DScene();
    win3D.repaint();
}

void DisplayScreen::displayBotPositions(vector<CPose3D> bot_path)
{
    win3D.get3DSceneAndLock();
    gl_keyframes->clear();
    for (size_t i = 0; i < bot_path.size(); i++) {
                CSetOfObjectsPtr obj = mrpt::opengl::stock_objects::CornerXYZSimple(0.3, 3);
                obj->setPose(bot_path[i]);
                gl_keyframes->insert(obj);
            }
    win3D.unlockAccess3DScene();
    win3D.repaint();
}

void DisplayScreen::displayBotPositions(CPose3D bot_pose)
{
    win3D.get3DSceneAndLock();
    gl_keyframes->clear();
    gl_cur_cam_corner->setPose(bot_pose);
    win3D.unlockAccess3DScene();
    win3D.repaint();
}

void DisplayScreen::updateGlobalMap(CObservation3DRangeScanPtr scan_data, 
        CPose3D curr_pose) {
    global_map.insertObservation(scan_data.pointer(), &curr_pose);
    win3D.get3DSceneAndLock();
    gl_points_map->loadFromPointsMap(&global_map);
    win3D.unlockAccess3DScene();
    win3D.repaint();
}

void DisplayScreen::initializeGlobalMap(CObservation3DRangeScanPtr scan_data) {
    global_map.clear();
    global_map.loadFromRangeScan(*scan_data);
    win3D.get3DSceneAndLock();
    gl_points_map->loadFromPointsMap(&global_map);
    win3D.unlockAccess3DScene();
    win3D.repaint();
}

DisplayScreen::~DisplayScreen() {
}

