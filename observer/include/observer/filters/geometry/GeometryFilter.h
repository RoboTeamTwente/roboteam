//
// Created by rolf on 26-10-20.
//

#ifndef RTT_GEOMETRYFILTER_H
#define RTT_GEOMETRYFILTER_H


#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>
class GeometryFilter {
public:
    bool process(const proto::SSL_GeometryData& geometryData);
    bool receivedFirstGeometry() const;
    proto::SSL_GeometryData getGeometry() const;
private:
    std::string lastGeometryString;
    proto::SSL_GeometryData combinedGeometry;
    std::map<unsigned int,proto::SSL_GeometryCameraCalibration> cameras;

};


#endif //RTT_GEOMETRYFILTER_H
