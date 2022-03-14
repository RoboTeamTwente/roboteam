#pragma once

#include <vector>

namespace rtt {

/* This object represents the camera calibration passed on
 * by vision. I myself have no idea what many of these fields mean. */
typedef struct CameraCalibration {
    unsigned int cameraId;
    float focalLength;
    float principalPointX;
    float principalPointY;
    float distortion;
    float q0;
    float q1;
    float q2;
    float q3;
    float tx;
    float ty;
    float tz;
    float derivedCameraWorldTx;
    float derivedCameraWorldTy;
    float derivedCameraWorldTz;
    unsigned int pixelImageWidth;
    unsigned int pixelImageHeight;
} CameraCalibration;

typedef std::vector<CameraCalibration> CameraCalibrations;

} // namespace rtt