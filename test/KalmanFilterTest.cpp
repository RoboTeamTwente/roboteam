//
// Created by rolf on 22-11-19.
//

#include <gtest/gtest.h>
#include "filters/KalmanFilter.h"

TEST(OneDimension, KalmanFilterTest){
    // see https://www.kalmanfilter.net/kalman1d.html for where I got the values from
    //create a simple one dimensional kalman filter.
    KalmanFilter<1,1>::Vector initialGuess={60.0};
    KalmanFilter<1,1>::Matrix variance={225.0};
    KalmanFilter<1,1> filter(initialGuess,variance);

    filter.B={1.0};
    filter.u.zeros();
    filter.F={1.0}; // we estimate the building height stays constant
    filter.Q.zeros();
    filter.H={1.0};// measurement scales linearly
    filter.R={25.0};// we can estimate a building up to 5m accurate (so variance is 5^2)
    // we test if the state of the filter is the initial guess and if the basestate and state match
    ASSERT_DOUBLE_EQ(filter.state()[0],60.0);
    ASSERT_DOUBLE_EQ(filter.basestate()[0],60.0);
    ASSERT_DOUBLE_EQ(filter.state()[0],filter.basestate()[0]);
    //the series of measurements
    double measurements[10]={48.54,47.11,55.01,55.15,49.89,40.85,46.72,50.05,51.27,49.95};
    //What our kalman filter should output after each update predict cycle.
    double outputs[10]={49.69,48.47,50.57,51.68,51.33,49.62,49.21,49.31,49.53,49.57};
    for (int i = 0; i < 10; ++i) {
        filter.z={measurements[i]};
        filter.update();
        filter.predict(true);
        //check if basestate and state match and if it matches predicted output.
        ASSERT_NEAR(filter.state()[0],outputs[i],0.01); //the values from internet have some rounding errors but are generally ok.
        ASSERT_NEAR(filter.basestate()[0],outputs[i],0.01);
        ASSERT_DOUBLE_EQ(filter.state()[0],filter.basestate()[0]);
    }
}