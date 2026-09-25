#ifndef LINO_AHRS_H
#define LINO_AHRS_H

// Madgwick's AHRS, on the board.
//
// WHY IT IS HERE RATHER THAN IN A NODE
//
// Until now a 9-axis robot published imu/data_raw and imu/mag and let
// imu_filter_madgwick pair them into imu/data. That pairing is a
// message_filters::sync_policies::ApproximateTime synchroniser five deep, so
// imu/data is not the rate of the IMU -- it is the rate of MATCHED PAIRS. Both
// topics cross a best-effort micro-ROS session, either can be dropped
// independently, and every lost magnetometer message costs an IMU sample.
//
// The bench has already shown what that costs, and shown it asymmetrically: on
// two legs where the board slowed down, /odom (which needs no partner) held
// 33 Hz while /imu/data (which does) fell to 10. A 1.5x loss in what the board
// sent became a 5x loss in what the EKF received.
//
// The board has both readings in the same 50 Hz cycle, from the same trigger,
// with no link between them and nothing to synchronise. Fusing here removes the
// dependency rather than tuning it -- and it removes two launch-time rules with
// it, because the board then always publishes imu/data and madgwick never runs.
//
// A part with its own fusion (BNO085) is simply the same architecture with the
// filter one level lower: it already returns a quaternion, so it does its best
// and this code stands aside. hasFusedOrientation() is the seam.
//
// FAITHFULNESS
//
// This is a port of imu_filter_madgwick's ImuFilter, not a new filter: the same
// gradient-descent step, the same gyro-drift compensation, the same ENU
// convention and the same defaults (gain 0.1, zeta 0.0). The intent is that
// moving the filter onto the board changes WHERE it runs and not WHAT it
// computes, so a bench result before and after is comparable.
//
// One deliberate difference: the reference uses the 1999 fast-inverse-square-root
// approximation, which was a win on hardware without an FPU. Every MCU this
// project targets has one, so this uses 1/sqrtf() -- more accurate, not less,
// and the ~0.17% the approximation costs sits far below the magnetometer noise
// this filter is weighing against.

#include <math.h>

// Standard gravity, which is what an accelerometer at rest reports and what has
// to come back out of the reading. Deliberately NOT SIM_IMU_GRAVITY: that is a
// simulation constant in sim_wheel.h, and the real fusion path must not depend on
// a simulation header for a physical constant. robot_localization uses the same
// value for its own `gravitational_acceleration`.
#ifndef AHRS_GRAVITY
#define AHRS_GRAVITY 9.80665f
#endif

#ifndef AHRS_GAIN
#define AHRS_GAIN 0.1f          // imu_filter_madgwick's default
#endif
#ifndef AHRS_ZETA
#define AHRS_ZETA 0.0f          // gyro drift compensation, off by default
#endif

class AHRS
{
public:
    // ENU, matching the `world_frame: enu` the launch has always passed:
    // gravity is +Z and the earth's field lies in the YZ plane.
    void setGain(float gain) { gain_ = gain; }
    void setZeta(float zeta) { zeta_ = zeta; }
    bool converged() const { return seeded_; }

    void quaternion(double &x, double &y, double &z, double &w) const
    {
        x = q1_; y = q2_; z = q3_; w = q0_;
    }

    // Gravity in the body frame, from an ARBITRARY orientation -- so a part that
    // fused on-chip (BNO085) gets its gravity removed by the same arithmetic,
    // from its own quaternion. Without this the BNO085 was the one path where
    // nobody subtracted it: the filter stands aside for that part, and the EKF is
    // told not to remove it either because the board normally has.
    static void gravityFrom(double qx, double qy, double qz, double qw,
                            float g, float &gx, float &gy, float &gz)
    {
        const float x = (float)qx, y = (float)qy, z = (float)qz, w = (float)qw;
        gx = g * 2.0f * (x * z - w * y);
        gy = g * 2.0f * (w * x + y * z);
        gz = g * (w * w - x * x - y * y + z * z);
    }

    // The gravity vector as this estimate believes it lies in the BODY frame,
    // so a caller can subtract it and publish a specific force that means
    // acceleration. This is what madgwick's remove_gravity_vector does, and the
    // EKF fuses ax and ay, so somebody has to do it.
    void gravity(float &gx, float &gy, float &gz, float g) const
    {
        gx = g * 2.0f * (q1_ * q3_ - q0_ * q2_);
        gy = g * 2.0f * (q0_ * q1_ + q2_ * q3_);
        gz = g * (q0_ * q0_ - q1_ * q1_ - q2_ * q2_ + q3_ * q3_);
    }

    // 9-axis. mx/my/mz in any consistent unit -- only the direction is used.
    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz, float dt)
    {
        if (!isfinite(mx) || !isfinite(my) || !isfinite(mz) ||
            (mx == 0.0f && my == 0.0f && mz == 0.0f)) {
            updateIMU(gx, gy, gz, ax, ay, az, dt);
            return;
        }
        if (dt <= 0.0f) return;

        float qd0, qd1, qd2, qd3;
        if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
            normalize3(ax, ay, az);
            normalize3(mx, my, mz);

            // The field's reference direction, from the current estimate.
            float hx, hy, hz;
            rotateAndScale(q0_, -q1_, -q2_, -q3_, mx, my, mz, hx, hy, hz);
            const float _2bxy = 4.0f * sqrtf(hx * hx + hy * hy);
            const float _2bz = 4.0f * hz;

            float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
            // ENU: gravity [0, 0, 1], field [0, bxy, bz]
            gradient(q0_, q1_, q2_, q3_, 0.0f, 0.0f, 2.0f, ax, ay, az, s0, s1, s2, s3);
            gradient(q0_, q1_, q2_, q3_, 0.0f, _2bxy, _2bz, mx, my, mz, s0, s1, s2, s3);
            normalize4(s0, s1, s2, s3);

            driftCompensate(s0, s1, s2, s3, dt, gx, gy, gz);
            fromGyro(gx, gy, gz, qd0, qd1, qd2, qd3);
            qd0 -= gain_ * s0;
            qd1 -= gain_ * s1;
            qd2 -= gain_ * s2;
            qd3 -= gain_ * s3;
        } else {
            fromGyro(gx, gy, gz, qd0, qd1, qd2, qd3);
        }

        integrate(qd0, qd1, qd2, qd3, dt);
        seeded_ = true;
    }

    // 6-axis. Levels roll and pitch against gravity; yaw is the gyro's own
    // integral and has nothing to anchor it, which is exactly why the EKF must
    // not fuse absolute yaw on a magnetometer-less robot.
    void updateIMU(float gx, float gy, float gz,
                   float ax, float ay, float az, float dt)
    {
        if (dt <= 0.0f) return;
        float qd0, qd1, qd2, qd3;
        if (ax != 0.0f || ay != 0.0f || az != 0.0f) {
            normalize3(ax, ay, az);
            float s0 = 0.0f, s1 = 0.0f, s2 = 0.0f, s3 = 0.0f;
            gradient(q0_, q1_, q2_, q3_, 0.0f, 0.0f, 2.0f, ax, ay, az, s0, s1, s2, s3);
            normalize4(s0, s1, s2, s3);
            driftCompensate(s0, s1, s2, s3, dt, gx, gy, gz);
            fromGyro(gx, gy, gz, qd0, qd1, qd2, qd3);
            qd0 -= gain_ * s0;
            qd1 -= gain_ * s1;
            qd2 -= gain_ * s2;
            qd3 -= gain_ * s3;
        } else {
            fromGyro(gx, gy, gz, qd0, qd1, qd2, qd3);
        }
        integrate(qd0, qd1, qd2, qd3, dt);
        seeded_ = true;
    }

private:
    float q0_ = 1.0f, q1_ = 0.0f, q2_ = 0.0f, q3_ = 0.0f;   // w, x, y, z
    float w_bx_ = 0.0f, w_by_ = 0.0f, w_bz_ = 0.0f;         // gyro bias estimate
    float gain_ = (float)AHRS_GAIN;
    float zeta_ = (float)AHRS_ZETA;
    bool seeded_ = false;

    static void normalize3(float &x, float &y, float &z)
    {
        const float n = sqrtf(x * x + y * y + z * z);
        if (n <= 0.0f) return;
        const float r = 1.0f / n;
        x *= r; y *= r; z *= r;
    }

    static void normalize4(float &a, float &b, float &c, float &d)
    {
        const float n = sqrtf(a * a + b * b + c * c + d * d);
        if (n <= 0.0f) return;
        const float r = 1.0f / n;
        a *= r; b *= r; c *= r; d *= r;
    }

    // Result is half as long as the input, as in the reference.
    static void rotateAndScale(float q0, float q1, float q2, float q3,
                               float _2dx, float _2dy, float _2dz,
                               float &rx, float &ry, float &rz)
    {
        rx = _2dx * (0.5f - q2 * q2 - q3 * q3) + _2dy * (q0 * q3 + q1 * q2) +
             _2dz * (q1 * q3 - q0 * q2);
        ry = _2dx * (q1 * q2 - q0 * q3) + _2dy * (0.5f - q1 * q1 - q3 * q3) +
             _2dz * (q0 * q1 + q2 * q3);
        rz = _2dx * (q0 * q2 + q1 * q3) + _2dy * (q2 * q3 - q0 * q1) +
             _2dz * (0.5f - q1 * q1 - q2 * q2);
    }

    static void gradient(float q0, float q1, float q2, float q3,
                         float _2dx, float _2dy, float _2dz,
                         float mx, float my, float mz,
                         float &s0, float &s1, float &s2, float &s3)
    {
        float f0, f1, f2;
        rotateAndScale(q0, q1, q2, q3, _2dx, _2dy, _2dz, f0, f1, f2);
        f0 -= mx; f1 -= my; f2 -= mz;

        s0 += (_2dy * q3 - _2dz * q2) * f0 + (-_2dx * q3 + _2dz * q1) * f1 +
              (_2dx * q2 - _2dy * q1) * f2;
        s1 += (_2dy * q2 + _2dz * q3) * f0 +
              (_2dx * q2 - 2.0f * _2dy * q1 + _2dz * q0) * f1 +
              (_2dx * q3 - _2dy * q0 - 2.0f * _2dz * q1) * f2;
        s2 += (-2.0f * _2dx * q2 + _2dy * q1 - _2dz * q0) * f0 +
              (_2dx * q1 + _2dz * q3) * f1 +
              (_2dx * q0 + _2dy * q3 - 2.0f * _2dz * q2) * f2;
        s3 += (-2.0f * _2dx * q3 + _2dy * q0 + _2dz * q1) * f0 +
              (-_2dx * q0 - 2.0f * _2dy * q3 + _2dz * q2) * f1 +
              (_2dx * q1 + _2dy * q2) * f2;
    }

    void driftCompensate(float s0, float s1, float s2, float s3, float dt,
                         float &gx, float &gy, float &gz)
    {
        // w_err = 2 q x s
        const float ex = 2.0f * q0_ * s1 - 2.0f * q1_ * s0 - 2.0f * q2_ * s3 + 2.0f * q3_ * s2;
        const float ey = 2.0f * q0_ * s2 + 2.0f * q1_ * s3 - 2.0f * q2_ * s0 - 2.0f * q3_ * s1;
        const float ez = 2.0f * q0_ * s3 - 2.0f * q1_ * s2 + 2.0f * q2_ * s1 - 2.0f * q3_ * s0;
        w_bx_ += ex * dt * zeta_;
        w_by_ += ey * dt * zeta_;
        w_bz_ += ez * dt * zeta_;
        gx -= w_bx_; gy -= w_by_; gz -= w_bz_;
    }

    void fromGyro(float gx, float gy, float gz,
                  float &qd0, float &qd1, float &qd2, float &qd3) const
    {
        qd0 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz);
        qd1 = 0.5f * (q0_ * gx + q2_ * gz - q3_ * gy);
        qd2 = 0.5f * (q0_ * gy - q1_ * gz + q3_ * gx);
        qd3 = 0.5f * (q0_ * gz + q1_ * gy - q2_ * gx);
    }

    void integrate(float qd0, float qd1, float qd2, float qd3, float dt)
    {
        q0_ += qd0 * dt;
        q1_ += qd1 * dt;
        q2_ += qd2 * dt;
        q3_ += qd3 * dt;
        normalize4(q0_, q1_, q2_, q3_);
    }
};

#endif  // LINO_AHRS_H
