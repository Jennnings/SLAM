/* 
 * File:   Tracker.cpp
 * Author: bhuvnesh
 * 
 * Created on 9 October, 2012, 8:09 PM
 */

#include "Tracker.h"

Tracker::Tracker() {
    tracker = CGenericFeatureTrackerAutoPtr(new CFeatureTracker_KL);
    tracker->enableTimeLogger(true); // Do time profiling.
    tracker->extra_params["add_new_features"] = 1; // track, AND ALSO, add new features
    tracker->extra_params["add_new_feat_min_separation"] = 25;
    tracker->extra_params["add_new_feat_max_features"] = 150;
    tracker->extra_params["add_new_feat_patch_size"] = 21;
    tracker->extra_params["minimum_KLT_response_to_add"] = 50;
    tracker->extra_params["check_KLT_response_every"] = 5; // Re-check the KLT-response to assure features are in good points.
    tracker->extra_params["minimum_KLT_response"] = 20; // Re-check the KLT-response to assure features are in good points.
    tracker->extra_params["update_patches_every"] = 0; // Update patches

    tracker->extra_params["window_width"] = 5;
    tracker->extra_params["window_height"] = 5;
}

Tracker::~Tracker() {
}

void Tracker::TrackFeatures(CObservation3DRangeScanPtr scan) {
    CImage theImg = scan->intensityImage; // current image
    tracker->trackFeatures(previous_image, theImg, tracked_features);

    // Remove those now out of the image plane and close to border of image
    CFeatureList::iterator itFeat = tracked_features.begin();
    while (itFeat != tracked_features.end()) {
        const TFeatureTrackStatus status = (*itFeat)->track_status;
        bool eras = (status_TRACKED != status && status_IDLE != status);
        if (!eras) {
            const float x = (*itFeat)->x;
            const float y = (*itFeat)->y;
            static const float MIN_DIST_MARGIN_TO_STOP_TRACKING = 10;
            if (x < MIN_DIST_MARGIN_TO_STOP_TRACKING || y < MIN_DIST_MARGIN_TO_STOP_TRACKING ||
                    x > (scan->cameraParamsIntensity.ncols - MIN_DIST_MARGIN_TO_STOP_TRACKING) ||
                    y > (scan->cameraParamsIntensity.nrows - MIN_DIST_MARGIN_TO_STOP_TRACKING)) {
                eras = true;
            }
        }
        if (eras) // Erase or keep?
            itFeat = tracked_features.erase(itFeat);
        else ++itFeat;
    }
}

void Tracker::calculate3DFeatures(CObservation3DRangeScanPtr scan) {

    // Creating list of 3D features in space, wrt current camera pose:
    for (CFeatureList::iterator itFeat = tracked_features.begin();
            itFeat != tracked_features.end(); ++itFeat) {
        // Pixel coordinates in the intensity image:
        const int int_x = (*itFeat)->x;
        const int int_y = (*itFeat)->y;
        // Convert to pixel coords in the range image:
        //  APPROXIMATION: Assume coordinates are equal (that's not exact!!)
        const int x = int_x;
        const int y = int_y;
        // Does this (x,y) have valid range data?
        const float d = scan->rangeImage(y, x);
        if (d > 0.05 && d < 10.0) {
            const size_t nPt = scan->rangeImage.cols() * y + x;
            curVisibleFeats[(*itFeat)->ID] = TPoint3D(scan->points3D_x[nPt], scan->points3D_y[nPt], scan->points3D_z[nPt]);
        }
    }
}

bool Tracker::calculatePoseMatrix() {
    // Estimate our current camera pose from feature2feature matching:
    if (!lastVisibleFeats.empty()) {
        TMatchingPairList corrs; // pairs of correspondences

        for (map<TFeatureID, TPoint3D>::const_iterator itCur = curVisibleFeats.begin();
                itCur != curVisibleFeats.end(); ++itCur) {
            map<TFeatureID, TPoint3D>::const_iterator itFound = lastVisibleFeats.find(itCur->first);
            if (itFound != lastVisibleFeats.end()) {
                corrs.push_back(TMatchingPair(
                        itFound->first,
                        itCur->first,
                        itFound->second.x, itFound->second.y, itFound->second.z,
                        itCur->second.x, itCur->second.y, itCur->second.z
                        ));
            }
        }

        if (corrs.size() >= 3) {
            // Find matchings:
            CPose3D relativePose;
            double scale;
            vector_int inliers_idx;
            const bool register_ok = mrpt::scanmatching::leastSquareErrorRigidTransformation6DRANSAC(//leastSquareErrorRigidTransformation6D(
                    corrs, // in correspondences
                    relativePose, // out relative pose
                    scale, // out scale : scale of the transformation should be around 1
                    inliers_idx,
                    3 // minimum inliers
                    );
//            cout << "correspondance points:" << corrs.size() << " inliers:"
//                    << inliers_idx.size() << endl;

            if (inliers_idx.size() == 0)
                cout << "ROBOT IS LOST!" << endl;

            // Checking if its a good match
            if (register_ok && std::abs(scale - 1.0) < 0.1) {
                //minimun motion of the robot for adding new location
                if ((relativePose.norm() > 0.10 ||
                        std::abs(relativePose.yaw()) > DEG2RAD(10) ||
                        std::abs(relativePose.pitch()) > DEG2RAD(10) ||
                        std::abs(relativePose.roll()) > DEG2RAD(10))) {
                    // Append new global pose of this key-frame:
                    const CPose3D new_keyframe_global = CPose3D(*camera_key_frames_path.rbegin()) + relativePose;
                    camera_key_frames_path.push_back(TPose3D(new_keyframe_global));
                    currentCamPose_wrt_last = CPose3D(); // It's (0,0,0) since the last key-frame is the current pose!
                    lastVisibleFeats = curVisibleFeats;
                    return true;
                } else {
                    currentCamPose_wrt_last = relativePose;
                    return false;
                }
            }
        }
    }
}

CPose3D Tracker::getCurrentGlobalPose(){
    return CPose3D(*camera_key_frames_path.rbegin())+currentCamPose_wrt_last;
}