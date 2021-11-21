#ifndef SYSTEMMODEL_HPP_
#define SYSTEMMODEL_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "thirdparty/kalman/include/kalman/LinearizedMeasurementModel.hpp"
#include "thirdparty/kalman/include/kalman/LinearizedSystemModel.hpp"
namespace Planar_Robot {
template <typename T>
class State : public Kalman::Vector<T, 10> {
  public:
    KALMAN_VECTOR(State, T, 10)
    //! X-position
    static constexpr size_t X = 0;
    //! Y-position
    static constexpr size_t Y = 1;
    //! Z-position
    static constexpr size_t Z = 2;
    //! Orientation
    static constexpr size_t Q_X = 3;
    static constexpr size_t Q_Y = 4;
    static constexpr size_t Q_Z = 5;
    static constexpr size_t Q_W = 6;

    //! Velocity
    static constexpr size_t V_X = 7;
    static constexpr size_t V_Y = 8;
    static constexpr size_t V_Z = 9;

  T& x()           { return (*this) [X];}
  T& y()           { return (*this) [Y];}
  T& z()           { return (*this) [Z];}
  T& qx()          { return (*this) [Q_X];}
  T& qy()          { return (*this) [Q_Y];}
  T& qz()          { return (*this) [Q_Z];}
  T& qw()          { return (*this) [Q_W];}
  T& vx()           { return (*this) [V_X];}
  T& vy()           { return (*this) [V_Y];}
  T& vz()           { return (*this) [V_Z];}
};

template <typename T>
class Control : public Kalman::Vector<T, 6> {
  public:
    KALMAN_VECTOR(Control, T, 6)
    //! Acceleration
    static constexpr size_t A_X = 0;
    static constexpr size_t A_Y = 1;
    static constexpr size_t A_Z = 2;
    //! Angular Rate
    static constexpr size_t W_X = 3;
    static constexpr size_t W_Y = 4;
    static constexpr size_t W_Z = 5;

    T a_x() const {
        return (*this)[A_X];
    }
    T a_y() const {
        return (*this)[A_Y];
    }
    T a_z() const {
        return (*this)[A_Z];
    }
    T w_x() const {
        return (*this)[W_X];
    }
    T w_y() const {
        return (*this)[W_Y];
    }
    T w_z() const {
        return (*this)[W_Z];
    }

    T& a_x() {
        return (*this)[A_X];
    }
    T& a_y() {
        return (*this)[A_Y];
    }
    T& a_z() {
        return (*this)[A_Z];
    }
    T& w_x() {
        return (*this)[W_X];
    }
    T& w_y() {
        return (*this)[W_Y];
    }
    T& w_z() {
        return (*this)[W_Z];
    }
};

template <typename T, template <class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef Planar_Robot::State<T> S;
    typedef Planar_Robot::Control<T> C;
    // Definition of (non-linear) state transition function
    S f(const S& x, const C& u) const {
        //! Predicted state vector after transition
        S x_;
        Eigen::Quaterniond Qwb;
        Eigen::Vector3d Pwb;
        Eigen::Vector3d Vw;
        double dt = 0.01;
        Qwb.x() = x.qx();
        Qwb.y() = x.qy();
        Qwb.z() = x.qz();
        Qwb.w() = x.qw();
        Pwb << x.x(), x.y(), x.z();
        Vw << x.vx(), x.vy(), x.vz();
        Eigen::Vector3d imu_gyro(u.w_x(), u.w_y(), u.w_z());
        Eigen::Vector3d imu_acc(u.a_x(), u.a_y(), u.a_z());

        Eigen::Quaterniond dq;
        Eigen::Vector3d half_newOriention = imu_gyro * dt / 2.0;
        dq.w() = 1;
        dq.x() = half_newOriention.x();
        dq.y() = half_newOriention.y();
        dq.z() = half_newOriention.z();

        Eigen::Vector3d acc_w = Qwb * imu_acc;
        Qwb = Qwb.normalized() * dq.normalized();
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw = Vw + acc_w * dt;
        // std::cout<<"position: "<<Pwb<<std::endl;
        x_.x() = Pwb(0);
        x_.y() = Pwb(1);
        x_.z() = Pwb(2);
        x_.qw() = Qwb.w();
        x_.qx() = Qwb.x();
        x_.qy() = Qwb.y();
        x_.qz() = Qwb.z();
        x_.vx() = Vw.x();
        x_.vy() = Vw.y();
        x_.vz() = Vw.z();
        return x_;
    }

  protected:
    void updataJacbians(const S& x, const C& u) {
        // TODO: more complicated and exact Jacbians matrix
        this->F.setIdentity();
        // partial derivative of x.x() w.r.t x.x()
        // this->F(S::X, S::X) = 1;
        // partial derivative of x.x() w.r.t x.theta()

        // partial derivative of x.y() w.r.t x.y()
        // this->F(S::Y, S::Y) = 1;
        // partial derivative of x.y() w.r.t. x.theta()

        // partia derivative of x.theta() w.r.t. x.theta()
        // this->F(S::THETA, S::THETA) = 1;

        // W = df/dw (Jacobian of state transition w.r.t. the noise)
        this->W.setIdentity();
        // float alpha1 = 0.02;
        // float alpha2 = 0.02;
        // float alpha3 = 0.02;
        // float alpha4 = 0.02;
        // Eigen::Matrix<float, 3, 2> V;
        // V(0, 0) = (-std::sin(x.theta()) + std::sin(x.theta() + u.dtheta()*0.01));
        // V(0, 1) = (u.v() * (std::sin(x.theta()) - std::sin(x.theta() + u.dtheta()*0.01)))  +
        //           (u.v() * (std::cos(x.theta() + u.dtheta()*0.01)))*0.01;
        // V(1, 1) = -(u.v() * (std::cos(x.theta()) - std::cos(x.theta() + u.dtheta()*0.01)))  +
        //           (u.v() * (std::sin(x.theta() + u.dtheta()*0.01)))*0.01;
        // V(1, 0) = (std::cos(x.theta()) - std::cos(x.theta() + u.dtheta()*0.01)) ;
        // V(1, 1) = 0.01;

        // Eigen::Matrix2f M;
        // M(0,0) = alpha1 * u.v_x() * u.v_y() + alpha2 * u.dtheta() * u.dtheta();
        // M(0,1) = 0.0;
        // M(1,0) = 0.0;
        // M(1,1) = alpha3 * u.v_x() * u.v_y() + alpha4 * u.dtheta() * u.dtheta();
        // this->W.setZero();
        // this->W = this->W + V * M * V.transpose();
    }
};
}  // namespace Planar_Robot
#endif
