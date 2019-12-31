//
// Created by rolf on 15-12-19.
//

#ifndef RTT_CAMERAFILTER_H
#define RTT_CAMERAFILTER_H


/*
 * This class relies on it's superclasses to set lastUpdateTime and framecount correctly!
 * lastMainUpdate is updated correctly if switchCamera is called for every update, as it should be
 */
class CameraFilter {
private:
    int frameCount = 0;
protected:
    double lastUpdateTime;
    double lastMainUpdateTime;
    int mainCamera = -1;

    /**
     * Checks if we should our main camera and does so if necessary.
     * @return true if we switched our main camera
     */
    bool switchCamera(int camera, double time);
public:
    explicit CameraFilter(double observationTime, int camera);
    /**
     * The time of the last observation which was processed by the filter
     * @return The time at which the filter was last updated
     */
    [[nodiscard]] double getLastUpdateTime() const;
    /**
     * The amount of observations the filter has processed
     * @return The amount of observations the filter processed
     */
    [[nodiscard]] int frames() const;
};


#endif //RTT_CAMERAFILTER_H
