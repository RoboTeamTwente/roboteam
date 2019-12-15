//
// Created by rolf on 15-12-19.
//

#include "CameraFilter.h"

int CameraFilter::frames() const {
    return frameCount;
}
double CameraFilter::getLastUpdateTime() const {
    return lastUpdateTime;
}
CameraFilter::CameraFilter(double observationTime, int camera) :
lastUpdateTime{observationTime},
lastMainUpdateTime{observationTime},
mainCamera{camera},
frameCount{1}{

}
bool CameraFilter::switchCamera(int camera, double time) {
    bool cameraSwitched = false;
    const double TIMEOUT_TIME=0.05;
    if (lastUpdateTime + TIMEOUT_TIME<time){
        // Check if this frame is actually from a new camera
        cameraSwitched = (camera != mainCamera);
        mainCamera = camera;
    }
    if (camera == mainCamera){
        lastMainUpdateTime=time;
        frameCount++;//We only count frames from the main camera
    }
    return cameraSwitched;
}
