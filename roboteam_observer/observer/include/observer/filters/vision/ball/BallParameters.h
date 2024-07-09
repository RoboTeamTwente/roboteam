#pragma once
#include "proto/State.pb.h"

class BallParameters {
   private:
    double ballRadius = 0.0215;
    double accSlide;
    double accRoll;
    double inertiaDistribution = 0.5;
    double chipDampingXYFirstHop;
    double chipDampingXYOtherHops;
    double chipDampingZ;

   public:
    BallParameters() : accSlide(0), accRoll(0), chipDampingXYFirstHop(0), chipDampingXYOtherHops(0), chipDampingZ(0) {}

    explicit BallParameters(const proto::SSL_GeometryData& geometryData) : BallParameters() {
        if (geometryData.models().has_straight_two_phase()) {
            accSlide = geometryData.models().straight_two_phase().acc_slide();
            accRoll = geometryData.models().straight_two_phase().acc_roll();
        }
        if (geometryData.models().has_chip_fixed_loss()) {
            chipDampingXYFirstHop = geometryData.models().chip_fixed_loss().damping_xy_first_hop();
            chipDampingXYOtherHops = geometryData.models().chip_fixed_loss().damping_xy_other_hops();
            chipDampingZ = geometryData.models().chip_fixed_loss().damping_z();
        }
    }
    double getBallRadius() const { return ballRadius; }
    double getAccSlide() const { return accSlide; }
    double getAccRoll() const { return accRoll; }
    double getInertiaDistribution() const { return inertiaDistribution; }
    double getChipDampingXYFirstHop() const { return chipDampingXYFirstHop; }
    double getChipDampingXYOtherHops() const { return chipDampingXYOtherHops; }
    double getChipDampingZ() const { return chipDampingZ; }
};