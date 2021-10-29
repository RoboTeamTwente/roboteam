//
// Created by rolf on 26-10-20.
//

#include "filters/geometry/GeometryFilter.h"

bool GeometryFilter::process(const proto::SSL_GeometryData& geometryData) {
    // We serialize the data to string and check if it is the same as the string of last message
    // There is no pretty way to compare protobufs unfortunately, so this is the most reliable/best we can do.
    // If so, there's no point in updating the geometry and we just return.
    std::string geomString = geometryData.SerializeAsString();
    if (geomString == lastGeometryString || !geometryData.has_field()) {
        return false;
    }
    lastGeometryString = geomString;
    // New message:
    // Add/overwrite any new camera info
    for (const auto& cameraCalib : geometryData.calib()) {
        cameras[cameraCalib.camera_id()] = cameraCalib;
    }
    // In our interpreted geometry we save all the latest camera information we received.
    //  This is relevant as camera geometry may be sent from multiple pc's in the case of a lot of camera's
    combinedGeometry.clear_calib();
    for (const auto& cam : cameras) {
        auto calibration = combinedGeometry.add_calib();
        calibration->CopyFrom(cam.second);
    }
    if (geometryData.has_field()) {
        combinedGeometry.mutable_field()->CopyFrom(geometryData.field());
    }
    return true;
}

bool GeometryFilter::receivedFirstGeometry() const { return !lastGeometryString.empty(); }

proto::SSL_GeometryData GeometryFilter::getGeometry() const { return combinedGeometry; }
