//
// Created by rolf on 23-09-21.
//

#ifndef RTT_CAMERAMAP_H
#define RTT_CAMERAMAP_H

#include "Camera.h"

class CameraMap{
public:
    void addCamera(const Camera& camera);
    const Camera& operator[](unsigned int id) const;
    void clear();
    [[nodiscard]] bool hasCamera(unsigned int id) const;
private:
    std::map<unsigned int,Camera> map;
};


#endif //RTT_CAMERAMAP_H
