//
// Created by rolf on 23-09-21.
//

#include "filters/vision/Camera.h"
#include <cfloat>

Camera::Camera(const proto::SSL_GeometryCameraCalibration &calibrationData) :
        id(calibrationData.camera_id()),
        pos(Eigen::Vector3d(calibrationData.derived_camera_world_tx(), calibrationData.derived_camera_world_ty(),
                            calibrationData.derived_camera_world_tz())),
        translation(Eigen::Vector3d(calibrationData.tx(), calibrationData.ty(), calibrationData.tz())),
        rotation(Eigen::Quaterniond(calibrationData.q3(), calibrationData.q0(), calibrationData.q1(),
                                    calibrationData.q2()).normalized()),
        principalPoint(Eigen::Vector2d(calibrationData.principal_point_x(), calibrationData.principal_point_y())),
        focalLength(calibrationData.focal_length()),
        distortion(calibrationData.distortion()) {
    //image width data is sometimes not present.
    if (calibrationData.has_pixel_image_width()) {
        imageWidth = calibrationData.pixel_image_width();
    } else {
        imageWidth = -1;
    }
    if (calibrationData.has_pixel_image_height()) {
        imageHeight = calibrationData.pixel_image_height();
    } else {
        imageHeight = -1;
    }
}

Eigen::Vector3d Camera::position() const {
    return pos / 1000.0;
}

Eigen::Quaterniond Camera::orientation() const {
    return rotation;
}

unsigned int Camera::getID() const {
    return id;
}

double Camera::radialDistortion(double radius) const {
    if (distortion <= DBL_MIN) {
        return radius;
    }
    double rd = 0;
    double a = distortion;
    double b = -9.0 * a * a * radius + a * sqrt(a * (12.0 + 81.0 * a * radius * radius));
    b = (b < 0.0) ? (-pow(b, 1.0 / 3.0)) : pow(b, 1.0 / 3.0);
    rd = pow(2.0 / 3.0, 1.0 / 3.0) / b - b / (pow(2.0 * 3.0 * 3.0, 1.0 / 3.0) * a);
    return rd;
}

double Camera::radialDistortionInv(double radius) const { return radius * (1.0 + radius * radius * distortion); }
Eigen::Vector2d Camera::radialDistortion(Eigen::Vector2d &imagePoint) const {
    double rd = radialDistortion(imagePoint.norm());
    imagePoint.normalize();  // We do in place normalization for speed
    Eigen::Vector2d distortedPoint = imagePoint * rd;
    return distortedPoint;
}
Eigen::Vector2d Camera::radialDistortionInv(Eigen::Vector2d &imagePoint) const {
    double rd = radialDistortionInv(imagePoint.norm());
    imagePoint.normalize();
    Eigen::Vector2d distortedPoint = imagePoint * rd;
    return distortedPoint;
}

Eigen::Vector2d Camera::fieldToImage(const Eigen::Vector3d &fieldPoint) const {
    // First transform the point into camera coordinates
    Eigen::Vector3d camCoorPoint = rotation * fieldPoint + translation;
    // Then project it onto the image:
    Eigen::Vector2d imageProjection(camCoorPoint.x() / camCoorPoint.z(), camCoorPoint.y() / camCoorPoint.z());
    // distort the point according to the camera distortion
    Eigen::Vector2d distortedProjection = radialDistortion(imageProjection);
    // Perform the image transformation by scaling and translating the final image point.
    return focalLength * distortedProjection + principalPoint;
}

Eigen::Vector3d Camera::imageToField(const Eigen::Vector2d &imagePoint, double assumedHeight) const {
    // Translate imagePoint to coordinatesystem to undistort (with distortion centre at 0)
    Eigen::Vector2d translatedImagePoint = (imagePoint - principalPoint) / focalLength;
    // Undistort the point
    Eigen::Vector2d undistortedPoint = radialDistortionInv(translatedImagePoint);

    // Now we create a 3d ray on the z-axis. We assume the imaging chip is unit distance behind the lens.
    Eigen::Vector3d ray(undistortedPoint.x(), undistortedPoint.y(), 1.0);
    // We compute the transformation from camera to field
    Eigen::Quaterniond fieldToCamInverse = rotation.inverse();
    Eigen::Vector3d rayInCam = fieldToCamInverse * ray;
    Eigen::Vector3d zeroInCam = fieldToCamInverse * (-translation);
    rayInCam.normalize();  // We need to normalize for the below calculation
    // Now compute the point where the ray intersects the field and return this point
    double t = rayPlaneIntersection(Eigen::Vector3d(0, 0, assumedHeight), Eigen::Vector3d(0, 0, 1), zeroInCam, rayInCam);
    return zeroInCam + rayInCam * t;
}
double Camera::rayPlaneIntersection(const Eigen::Vector3d& planeOrigin,const Eigen::Vector3d& planeNormal,const Eigen::Vector3d& rayOrigin,const Eigen::Vector3d& rayDirection){
    return (-planeNormal).dot(rayOrigin - planeOrigin) / (planeNormal.dot(rayDirection));

}
bool Camera::isPositionVisible(const Eigen::Vector3d& fieldPoint, double marginFactor) const {
    if(imageWidth == -1 || imageHeight == -1){
        return true;
    }
    Eigen::Vector2d imagePos = fieldToImage(fieldPoint);
    double margin = marginFactor * std::max(imageWidth,imageHeight);
    double minWidth = margin;
    double maxWidth = imageWidth - margin;
    double minHeight = margin;
    double maxHeight = imageHeight - margin;
    return imagePos.x() >= minWidth && imagePos.x() <= maxWidth && imagePos.y() >= minHeight && imagePos.y() <= maxHeight;
}

Eigen::Vector2d Camera::linearProjectToHorizontalPlane(const Eigen::Vector3d& objectPos, double planeHeight) const {
    Eigen::Vector3d origin = position();
    Eigen::Vector3d rayDirection = objectPos-origin;
    Eigen::Vector3d planeOrigin(0,0,planeHeight);
    Eigen::Vector3d planeNormal(0,0,1);
    double t = rayPlaneIntersection(planeOrigin,planeNormal,origin,rayDirection);
    Eigen::Vector3d groundPos = origin + t*rayDirection;
    return Eigen::Vector2d(groundPos.x(),groundPos.y());
}
