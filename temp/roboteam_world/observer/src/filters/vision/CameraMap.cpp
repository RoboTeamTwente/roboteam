//
// Created by rolf on 23-09-21.
//

#include "filters/vision/CameraMap.h"

void CameraMap::addCamera(const Camera &camera) {
    map.insert({camera.getID(),camera});
}

const Camera &CameraMap::operator[](unsigned int id) const {
    return map.at(id);
}

void CameraMap::clear() {
    map.clear();
}

bool CameraMap::hasCamera(unsigned int id) const {
    return map.contains(id);
}
