//
// Created by rolf on 23-09-21.
//

#ifndef RTT_CAMERA_H
#define RTT_CAMERA_H

#include <Eigen/Dense>
#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>

/**
 * @author Rolf
 * @brief Class which represents a camera and it's relevant parameters.
 * Note that one should be careful handling units with this class, as the internal implementation mirrors that of ssl-vision, which is in mm.
 */
class Camera {
public:
    /**
     * @brief Construct a camera from the calibration data received in a ssl-vision message
     */
    explicit Camera(const proto::SSL_GeometryCameraCalibration& calibrationData);
    /**
     * @return The 3d position of the camera in metres [m]
     */
    [[nodiscard]] Eigen::Vector3d position() const;
    /**
     * @return The quaternion corresponding to the rotation matrix of the estimated camera rotation
     */
    [[nodiscard]] Eigen::Quaterniond orientation() const;
    /**
     * @return The ID of the camera
     */
    [[nodiscard]] unsigned int getID() const;

    /**
     * All vectors are in meters
     * @param objectPos 3d position of object to be project to plane
     * @param planeHeight height of the plane,
     * @return The vector in meters on the plane
     */
    [[nodiscard]] Eigen::Vector2d linearProjectToHorizontalPlane(const Eigen::Vector3d& objectPos,double planeHeight) const;
    /**
     * @brief !! Position is in millimeters, NOT meters!!
     * Checks if the position is visible. The marginFactor is subtracted from the image boundaries in pixels on each side
     * And then it is checked if the coordinate produced falls within the acceptable range.
     * E.g. if marginFactor is 0.1 and the image is 1280x1024 128 pixels are substracted from each side and only the inner
     * region is considered visible.
     * This function returns true if the image width and height are unknown
     */
    [[nodiscard]] bool isPositionVisible(const Eigen::Vector3d& fieldPoint, double marginFactor = 0.0) const;

    // All of the below 6 functions are just Eigen3 implementations of the logic in SSL-Vision, so nothing here is really 'created' by us.
    // If you have problems understanding it I recommend looking further at camera_calibration.h in ssl-vision or to send
    // me a message so I can help explaining it. Note the SSL-vision code may change, so compatibility can break here
    /**
     * @brief maps 3d field points to points on the camera image coordinates
     * @param fieldPoint  3d point to be mapped to image !!! in millimeters !!!
     * @return the pixel coordinates of the 3d point that was mapped.
     */
    [[nodiscard]] Eigen::Vector2d fieldToImage(const Eigen::Vector3d& fieldPoint) const;
    /**
     * !!RETURNS position in millimeter!!
     * @param imagePoint point in image that is used to determine object position
     * @param assumedHeight height that object is assumed to be detected at
     * @return 3d Position of the point in world coordinates !!! in millimeters !!!
     */
    [[nodiscard]] Eigen::Vector3d imageToField(const Eigen::Vector2d& imagePoint, double assumedHeight) const;



private:
    [[nodiscard]] Eigen::Vector2d radialDistortion(Eigen::Vector2d& imagePoint) const;
    [[nodiscard]] double radialDistortion(double radius) const;
    [[nodiscard]] double radialDistortionInv(double radius) const;
    [[nodiscard]] Eigen::Vector2d radialDistortionInv(Eigen::Vector2d& imagePoint) const;

    static double rayPlaneIntersection(const Eigen::Vector3d& planeOrigin,const Eigen::Vector3d& planeNormal,const Eigen::Vector3d& rayOrigin,const Eigen::Vector3d& rayDirection);
    unsigned int id;
    Eigen::Vector3d pos;
    Eigen::Vector3d translation;
    Eigen::Quaterniond rotation;
    Eigen::Vector2d principalPoint;
    double focalLength;
    double distortion;
    int imageWidth;
    int imageHeight;
};


#endif //RTT_CAMERA_H
