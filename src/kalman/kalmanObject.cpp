//
// Created by kjhertenberg on 13-5-19.
//

#include <roboteam_world/kalman/kalmanObject.h>
#include <DetectionRobot.pb.h>
namespace rtt {

    void kalmanObject::kalmanUpdateK() {

        if (this->comparisonCount < MAXCOMPARISONS) {

            /*
             * P = FPF^T+Q
             * S = R + HPH^T
             * K = PHS^-1
             * P = (I-KH)P(I-KH)^T+KRK^T
             */
            arma::fmat::fixed<STATEINDEX, STATEINDEX> F_transpose = this->F.t();
            arma::fmat::fixed<STATEINDEX, STATEINDEX> P_predict = (this->F * this->P * F_transpose) + this->Q;
            arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> H_transpose = this->H.t();
            arma::fmat::fixed<OBSERVATIONINDEX, OBSERVATIONINDEX> S = this->R + (this->H * P_predict * H_transpose);
            arma::fmat::fixed<OBSERVATIONINDEX, OBSERVATIONINDEX> S_inverse = S.i();
            arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> K_new = P_predict * H_transpose * S_inverse;
            arma::fmat::fixed<OBSERVATIONINDEX, STATEINDEX> K_new_transpose = K_new.t();
            arma::fmat::fixed<STATEINDEX, STATEINDEX> IKH = this->I - K_new * this->H;
            arma::fmat::fixed<STATEINDEX, STATEINDEX> IKH_transpose = IKH.t();
            arma::fmat::fixed<STATEINDEX, STATEINDEX> P_new = IKH * P_predict * IKH_transpose + K_new * this->R * K_new_transpose;

            //See if the K has changed over the iteration, if it hasn't after 100 iterations then stop calculating it
            float K_Diff_Max = (this->K - K_new).max();
            float K_Diff_Min = (this->K - K_new).min();
            int same = 0;
            if ((K_Diff_Max < KMARGIN) and (K_Diff_Min > -KMARGIN)){
                same += 1;
            }

            if (same == STATEINDEX * OBSERVATIONINDEX) {
                this->comparisonCount += 1;
            } else {
                this->comparisonCount = 0;
            }

            for (arma::uword i = 0; i < STATEINDEX; ++i) {
                for (arma::uword j = 0; j < OBSERVATIONINDEX; ++j) {
                    this->K(i, j) = K_new(i, j);
                }
                for (arma::uword k = 0; k < STATEINDEX; ++k) {
                    this->P(i,k) = P_new(i,k);
                }
            }
        }
    }

    void kalmanObject::kalmanUpdateX() {

        this->invisibleCounter += 1;
        if (this->invisibleCounter> DISAPPEARTIME&&this->exists){
            if (this->id!=-1) {
                std::cout << "Removing bot: " << this->id<< std::endl;
            }
        }
        if (this->invisibleCounter > DISAPPEARTIME || !this->exists) {
            this->exists = false;
        } else {
            // X_predict = FX_current
            // Y = Z - HX_predict
            // X_new = X_predict + Ky

            arma::fvec::fixed<STATEINDEX> X_predict = this->F * this->X;

            arma::fmat::fixed<OBSERVATIONINDEX, 1> Y = this->Z - (this->H * X_predict);

            arma::fvec::fixed<STATEINDEX> X_new = X_predict + (this->K * Y);

            for (arma::uword i = 0; i < STATEINDEX; ++i) {
                this->X(i) = X_new(i);
            }

        }
    }

    void kalmanObject::kalmanUpdateZ(roboteam_proto::DetectionRobot robot, double timeStamp, uint cameraID) {
        //if the new data is a certain distance from the old/predicted data, it's considered a ghost and ignored
        if (this->exists){
            float errorx = robot.pos().x()-this->X(0);
            float errory = robot.pos().y()-this->X(2);
            if (errorx*errorx+errory*errory >= 0.2*0.2){
                return;
            }
        }
        //if the object comes into being, make the observation it's state, (to prevent jumping)
        if (!this->exists){
            std::cout<<"Adding bot: "<<robot.robot_id()<<std::endl;
            this->pastObservation.clear();
            this->X(0) = robot.pos().x();
            this->X(2) = robot.pos().y();
        }
        Position average = calculatePos(robot.pos(), robot.orientation(), cameraID);
        this->cameraId = cameraID;
        this->id= robot.robot_id();
        this->Z(0) = average.x;
        this->Z(1) = average.y;
        this->omega = (average.rot - this->orientation)/(timeStamp-this->observationTimeStamp);
        this->orientation = average.rot;
        this->observationTimeStamp = timeStamp;
        this->invisibleCounter = 0;
        this->exists = true;
    }

    Position kalmanObject::kalmanGetPos() const{
        return {this->X(0), this->X(2), this->orientation};
    }

    Position kalmanObject::kalmanGetVel() const{
        return {this->X(1), this->X(3), this->omega};
    }

    float kalmanObject::getK(){
        return this->K(0, 0);
    }

    bool kalmanObject::getExistence() const{
        return this->exists;
    }

    roboteam_proto::WorldRobot kalmanObject::as_message() const{
        roboteam_proto::WorldRobot msg;
        Position pos = kalmanGetPos();
        Position vel = kalmanGetVel();
        msg.set_id(id);
        msg.mutable_pos()->set_x(pos.x);
        msg.mutable_pos()->set_y(pos.y);
        msg.set_angle(limitRotation(pos.rot));
        msg.mutable_vel()->set_x(vel.x);
        msg.mutable_vel()->set_y(vel.y);
        msg.set_w(vel.rot);
        return msg;
    }

    double kalmanObject::limitRotation(double rotation) const{
        double constRot=fmod(rotation+M_PI, 2*M_PI)-M_PI;
        if (constRot<-M_PI||constRot>=M_PI){
            return -M_PI+std::numeric_limits<float>::epsilon();
        }
        return constRot;
    }

    Position kalmanObject::calculatePos(Vector2 pos, float rot, uint camID){
        if (camID == this->cameraId){
            this->pastObservation.clear();
            return {pos.x, pos.y, rot};
        } else {
            this->pastObservation[camID] = {pos.x, pos.y, rot};
            Position average = {0, 0, 0};
            for (auto obs : pastObservation) {
                average.x += obs.second.x;
                average.y += obs.second.y;
                average.rot += obs.second.rot;
            }
            float amount = this->pastObservation.size();
            average.y /= amount;
            average.x /= amount;
            average.rot /= amount;
            return average;
        }
    }


}