//
// Created by rolf on 26-10-20.
//

#ifndef RTT_GEOMETRYFILTER_H
#define RTT_GEOMETRYFILTER_H


#include <proto/messages_robocup_ssl_geometry.pb.h>
/**
 * @brief A class which filters the geometry from multiple camera's/sources and merges it to a single geometry correctly.
 */
class GeometryFilter {
public:
    /**
     * Updates the filter, replacing the field geometry data and adding any camera's to the list of known camera calibrations
     * @param geometryData data to update the filter with
     * @return true if the geometry changed/was updated, false if this geometry was already known
     */
    bool process(const proto::SSL_GeometryData& geometryData);
    /**
     * @return true if a geometry message was received, false otherwise
     */
    bool receivedFirstGeometry() const;
    /**
     * @return The most recent filtered geometry
     */
    proto::SSL_GeometryData getGeometry() const;
private:
    std::string lastGeometryString;
    proto::SSL_GeometryData combinedGeometry;
    std::map<unsigned int,proto::SSL_GeometryCameraCalibration> cameras;

};


#endif //RTT_GEOMETRYFILTER_H
