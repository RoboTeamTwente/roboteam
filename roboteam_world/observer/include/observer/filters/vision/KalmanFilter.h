#ifndef RTT_KALMANFILTER_H
#define RTT_KALMANFILTER_H

#include <Eigen/Dense>
/**
 * A class for generic standard basic linear Kalman Filters using doubles and dense matrixes.
 * Variable names are the same as on wikipedia: https://en.wikipedia.org/wiki/Kalman_filter
 * @author Rolf
 * @date 5 November 2019
 * @param STATEDIM dimension of state vector
 * @param OBSDIM dimension of measurement vector
 */
template <int STATEDIM, int OBSDIM>
class KalmanFilter {
 public:
  typedef Eigen::Matrix<double, STATEDIM, STATEDIM> Matrix;
  typedef Eigen::Matrix<double, OBSDIM, STATEDIM> MatrixO;
  typedef Eigen::Matrix<double, OBSDIM, OBSDIM> MatrixOO;
  typedef Eigen::Matrix<double, STATEDIM, OBSDIM> MatrixSO;
  typedef Eigen::Matrix<double, STATEDIM, 1> Vector;
  typedef Eigen::Matrix<double, OBSDIM, 1> VectorO;

 private:
  Vector X;  // State of the system
  Matrix P;  // covariance matrix of the system (how sure are we of the state?)

 public:
  Matrix F;    // Forward model/state update matrix. Essentially a linear model of what we predict the next state will be
  MatrixO H;   // Observation model/ states how we can interpret observation as our state
  Matrix Q;    // Covariance of the process noise. (Amount of "Random Forces" we can expect in the process)
  MatrixOO R;  // Observation Noise Covariance. Keeps track of how noisy the observations are.

  // These are only really used in extended Kalman Filters or when we add control input.
  Matrix B;  // State transition jacobian

  VectorO y; //Innovation. Not strictly necessary to store but often used to measure performance
  //implicit constructor
  KalmanFilter(){
    F = Matrix::Identity();
    H = MatrixO::Identity();
    Q = Matrix::Zero();
    R = MatrixOO::Zero();

    B = Matrix::Identity();
    y = VectorO::Zero();
  }
  /**
   * Constructs a Kalman Filter which starts with initial values and noise estimates
   * By default we simply have every column/row independent and no noises/perfect measurements
   * @param x initial state vector
   * @param p initial covariance vector
   */
  explicit KalmanFilter(const Vector& x, const Matrix& p) : X(x), P(p) {
    F = Matrix::Identity();
    H = MatrixO::Identity();
    Q = Matrix::Zero();
    R = MatrixOO::Zero();

    B = Matrix::Identity();
    y = VectorO::Zero();
  };
  /**
   * Predict the next state using forward model, updating the state and covariance estimates
   * @param permanentUpdate if set to true, permanently updates the filter's state.
   * Otherwise, the prediction is only stored locally as a prediction
   */
  void predict() {
    X = F * X;
    P = F * P * F.transpose() + Q;
  };
  /**
   * Predict the next state using forward model, updating the state and covariance estimates
   * @param permanentUpdate if set to true, permanently updates the filter's state.
   * Otherwise, the prediction is only stored locally as a prediction
   */
  void predict(const Vector& u) {
    X = F * X + B * u;
    P = F * P * F.transpose() + Q;
  };
  /**
   * @brief Updates the filter with observation z
   * Is optimized to perform better on small matrices.
   * The inverse method for this is better suited for small matrices
   * and should always be used if the number of state dimensions is <= 4
   */
  void update(const VectorO& z) {
    // Compute innovation (error between measurement and state)
    y = z - (H * X);
    // Variance of innovation
    MatrixOO S = H * P * H.transpose() + R;
    // compute kalman gain. For small matrices, Eigen's inverse function is most efficient
    MatrixSO K = P * H.transpose() * S.inverse();
    // update state with prediction
    X = X + K * y;
    // update covariance
    P -= K * H * P;
  };

  /**
   * Returns the state of the system.
   * @return  estimated state of the system
   */
  const Vector& state() const { return X; }
  /**
   * Returns the covariance matrix of the system..
   * @return  estimated covariance matrix of the system
   */
  const Matrix& covariance() const { return P; }

  /**
   * Manually set state[index]=value, modifying the state of the filter. Should only be used sparsely.
   * This is only recommended when you are somehow using
   * a filter which extrapolates a bit too aggressively and goes ' out of bounds' (for example for angular filters) after some observations.
   * @param index. Index of the state to be modified
   * @param value. Value to set state[index] to
   */
  void modifyState(int index, double value) { X(index) = value; }

  /**
   * Completely resets the state. Note this does NOT reset the covariance!! Use with care
   * @param state
   */
  void setState(const Vector& state){
    X = state;
  }

  /**
   * Completely resets the covariance. Use with care!
   * @param covariance
   */
  void setCovariance(const Matrix& covariance){
    P = covariance;
  }
};

#endif  // RTT_KALMANFILTER_H
