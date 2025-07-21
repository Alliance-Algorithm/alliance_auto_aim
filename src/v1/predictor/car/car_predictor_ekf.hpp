#pragma once

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/armor_id.hpp"
#include "util/ekf.hpp"
#include "util/math.hpp"
#include <vector>

namespace world_exe::predictor {

class CarPredictEkf : public world_exe::util::Ekf<11, 4, CarPredictEkf> {
public:
    CarPredictEkf() {
        X_k << 2.0, 0.0, 0.0, 0.0, -0.3, -0.3, -0.3, 0.2, 0.2, std::numbers::pi, 0.0;

        P_k.diagonal() << 1., 1., 1., 1., 1., 0., 0., 1., 1., 1., 1.;
    }

    inline void set_second_armor() { second_armor_flag = true; }

    inline std::vector<data::ArmorGimbalControlSpacing> get_predict_output_armor(
        const enumeration::ArmorIdFlag& id, const double& sec) const {
        std::vector<data::ArmorGimbalControlSpacing> target_armors;
        const auto model_output = X_k;
        const auto vx           = model_output(1);
        const auto vy           = model_output(3);

        const double car_x = model_output(0) + vx * sec;
        const double car_y = model_output(2) + vy * sec;

        const double car_z = model_output(4);
        const double z1    = model_output(5);
        const double z2    = model_output(6);
        const double r1    = model_output(7);
        const double r2    = model_output(8);
        double model_yaw   = model_output(9) + model_output(10) * sec;

        // TODO: 把 pitch 与 z 关联起来
        const double pitch1 = -15. / 180. * std::numbers::pi;
        const double pitch2 = -15. / 180. * std::numbers::pi;

        if (side_flag_) {
            const auto armor1 = data::ArmorGimbalControlSpacing { id,
                { car_x + r1 * std::cos(model_yaw), car_y + r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor1);

            model_yaw += std::numbers::pi / 2.;
            const auto armor2 = data::ArmorGimbalControlSpacing { id,
                { car_x + r2 * std::cos(model_yaw), car_y + r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor2);

            model_yaw += std::numbers::pi / 2.;
            const auto armor3 = data::ArmorGimbalControlSpacing { id,
                { car_x + r1 * std::cos(model_yaw), car_y + r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor3);

            model_yaw += std::numbers::pi / 2.;
            const auto armor4 = data::ArmorGimbalControlSpacing { id,
                { car_x + r2 * std::cos(model_yaw), car_y + r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor4);
        } else {
            const auto armor1 = data::ArmorGimbalControlSpacing { id,
                { car_x + r2 * std::cos(model_yaw), car_y + r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor1);

            model_yaw += std::numbers::pi / 2.;
            const auto armor2 = data::ArmorGimbalControlSpacing { id,
                { car_x + r1 * std::cos(model_yaw), car_y + r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor2);

            model_yaw += std::numbers::pi / 2.;
            const auto armor3 = data::ArmorGimbalControlSpacing { id,
                { car_x + r2 * std::cos(model_yaw), car_y + r2 * std::sin(model_yaw), z2 },
                util::math::euler_to_quaternion(model_yaw, pitch2, 0.0) };
            target_armors.emplace_back(armor3);

            model_yaw += std::numbers::pi / 2.;
            const auto armor4 = data::ArmorGimbalControlSpacing { id,
                { car_x + r1 * std::cos(model_yaw), car_y + r1 * std::sin(model_yaw), z1 },
                util::math::euler_to_quaternion(model_yaw, pitch1, 0.0) };
            target_armors.emplace_back(armor4);
        }

        return target_armors;
    }

private:
    friend class world_exe::util::Ekf<11, 4, CarPredictEkf>;

    //           0,   1, 2,   3, 4,   5,   6,  7,  8, 9,10
    // x分别代表[ x, v_x, y, v_y, z, z_1, z_2, r1, r2, θ, ω ]
    // z分别代表[ θ, yaw, pitch, distance ]
    inline ZVec process_z(const ZVec& z_k) {
        ZVec processed_z { z_k };
        if (z_k(0) < 0.) processed_z(0) += std::numbers::pi * 2.;

        double offset { processed_z(0) - last_theta_ };
        if (std::abs(offset) >= switch_angle_difference_) {
            side_flag_ = !side_flag_;
        } else offset = 0.;

        last_theta_ = processed_z(0);

        if (side_flag_) X_k(5) = z_k(3) * std::cos(z_k(1)) * -std::sin(z_k(2));
        else X_k(6) = z_k(3) * std::cos(z_k(1)) * -std::sin(z_k(2));

        if (X_k(9) < 0.) X_k(9) += std::numbers::pi * 2.;

        X_k(9) += offset;
        return processed_z;
    };

    inline XVec normalize_x(const XVec& x_k) {
        normalized_x    = x_k;
        auto camera_yaw = x_k(9);
        while (camera_yaw < 0.)
            camera_yaw += 2 * std::numbers::pi;
        while (camera_yaw >= 2 * std::numbers::pi)
            camera_yaw -= 2 * std::numbers::pi;

        normalized_x(9) = camera_yaw;
        return normalized_x;
    };

    inline XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) {
        const double x     = X_k(0) + X_k(1) * dt;
        const double y     = X_k(2) + X_k(3) * dt;
        const double z     = (X_k(5) + X_k(6)) / 2.;
        const double theta = X_k(9) + X_k(10) * dt;

        f_ << x, X_k(1), y, X_k(3), z, X_k(5), X_k(6), X_k(7), X_k(8), theta, X_k(10);

        return f_;
    };

    inline ZVec h(const XVec& x_k_n, const VVec&) {
        double r, z;
        if (side_flag_) {
            r = x_k_n(7);
            z = x_k_n(5);
        } else {
            r = x_k_n(8);
            z = x_k_n(6);
        }

        const double theta    = x_k_n(9);
        const double armor_x  = x_k_n(0) + r * std::cos(theta);
        const double armor_y  = x_k_n(2) + r * std::sin(theta);
        const double yaw      = std::atan(armor_y / armor_x);
        const double pitch    = -std::atan(z / armor_x);
        const double distance = std::sqrt(armor_x * armor_x + armor_y * armor_y + z * z);
        h_ << x_k_n(9), yaw, pitch, distance;
        return h_;
    };

    inline AMat A(const XVec&, const XVec&, const XVec&, const double& dt) {
        // clang-format off
            A_ << 1., dt, 0., 0., 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 1., 0., 0., 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 0., 1., dt, 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 1., 0.,  0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0.5,  0.5, 0., 0., 0., 0., 
                  0., 0., 0., 0., 0.,  1.,   0., 0., 0., 0., 0., 
                  0., 0., 0., 0., 0.,  0.,   1., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0.,  0.,   0., 1., 0., 0., 0.,
                  0., 0., 0., 0., 0.,  0.,   0., 0., 1., 0., 0., 
                  0., 0., 0., 0., 0.,  0.,   0., 0., 0., 1., dt, 
                  0., 0., 0., 0., 0.,  0.,   0., 0., 0., 0., 1.;
        // clang-format on
        return A_;
    };

    inline WMat W(const XVec&, const XVec&, const XVec&) const { return W_; };

    inline HMat H(const XVec& x_k_n, const VVec&) {
        const double theta = x_k_n(9);

        if (side_flag_) {
            const double r         = x_k_n(7);
            const double z         = x_k_n(5);
            const double armor_x   = x_k_n(0) + r * std::cos(theta);
            const double armor_y   = x_k_n(2) + r * std::sin(theta);
            const double armor_x_2 = armor_x * armor_x;
            const double armor_y_2 = armor_y * armor_y;

            const double d_x_2_base = 1.0 / (1.0 + armor_y_2 / armor_x_2);
            const double d_x_2_0    = -d_x_2_base * armor_y / armor_x_2;
            const double d_x_2_2    = d_x_2_base / armor_x;
            const double d_x_2_7 =
                d_x_2_base * (-armor_y / armor_x_2 * std::cos(theta) + std::sin(theta) / armor_x);
            const double d_x_2_9 = d_x_2_base * r
                * (std::sin(theta) * armor_y / armor_x_2 + std::cos(theta) / armor_x);

            const double d_x_3_base = 1.0 / armor_x_2 / (1.0 + z * z / armor_x_2);
            const double d_x_3_0    = d_x_3_base * z;
            const double d_x_3_5    = -d_x_3_base * armor_x;
            const double d_x_3_7    = d_x_3_0 * std::cos(theta);
            const double d_x_3_9    = -d_x_3_0 * r * std::sin(theta);

            const double d_x_4_base = 1.0 / std::sqrt(z * z + armor_x_2 + armor_y_2);
            const double d_x_4_0    = d_x_4_base * armor_x;
            const double d_x_4_2    = d_x_4_base * armor_y;
            const double d_x_4_5    = d_x_4_base * z;
            const double d_x_4_7 =
                d_x_4_base * (std::sin(theta) * armor_y + std::cos(theta) * armor_x);
            const double d_x_4_9 =
                d_x_4_base * r * (std::cos(theta) * armor_y - std::sin(theta) * armor_x);

            // clang-format off
            H_ <<      0., 0.,      0., 0., 0.,      0., 0.,      0., 0.,      1., 0.,
                  d_x_2_0, 0., d_x_2_2, 0., 0.,      0., 0., d_x_2_7, 0., d_x_2_9, 0., 
                  d_x_3_0, 0.,      0., 0., 0., d_x_3_5, 0., d_x_3_7, 0., d_x_3_9, 0.,
                  d_x_4_0, 0., d_x_4_2, 0., 0., d_x_4_5, 0., d_x_4_7, 0., d_x_4_9, 0.;

            // clang-format on
        } else {
            const double r         = x_k_n(8);
            const double z         = x_k_n(6);
            const double armor_x   = x_k_n(0) + r * std::cos(theta);
            const double armor_y   = x_k_n(2) + r * std::sin(theta);
            const double armor_x_2 = armor_x * armor_x;
            const double armor_y_2 = armor_y * armor_y;

            const double d_x_2_base = 1.0 / (1.0 + armor_y_2 / armor_x_2);
            const double d_x_2_0    = -d_x_2_base * armor_y / armor_x_2;
            const double d_x_2_2    = d_x_2_base / armor_x;
            const double d_x_2_8 =
                d_x_2_base * (-armor_y / armor_x_2 * std::cos(theta) + std::sin(theta) / armor_x);
            const double d_x_2_9 = d_x_2_base * r
                * (std::sin(theta) * armor_y / armor_x_2 + std::cos(theta) / armor_x);

            const double d_x_3_base = 1.0 / armor_x_2 / (1.0 + z * z / armor_x_2);
            const double d_x_3_0    = d_x_3_base * z;
            const double d_x_3_6    = -d_x_3_base * armor_x;
            const double d_x_3_8    = d_x_3_0 * std::cos(theta);
            const double d_x_3_9    = -d_x_3_0 * r * std::sin(theta);

            const double d_x_4_base = 1.0 / std::sqrt(z * z + armor_x_2 + armor_y_2);
            const double d_x_4_0    = d_x_4_base * armor_x;
            const double d_x_4_2    = d_x_4_base * armor_y;
            const double d_x_4_6    = d_x_4_base * z;
            const double d_x_4_8 =
                d_x_4_base * (std::sin(theta) * armor_y + std::cos(theta) * armor_x);
            const double d_x_4_9 =
                d_x_4_base * r * (std::cos(theta) * armor_y - std::sin(theta) * armor_x);

            // clang-format off
            H_ <<      0., 0.,      0., 0., 0., 0.,      0., 0.,      0.,      1., 0.,
                  d_x_2_0, 0., d_x_2_2, 0., 0., 0.,      0., 0., d_x_2_8, d_x_2_9, 0., 
                  d_x_3_0, 0.,      0., 0., 0., 0., d_x_3_6, 0., d_x_3_8, d_x_3_9, 0.,
                  d_x_4_0, 0., d_x_4_2, 0., 0., 0., d_x_4_6, 0., d_x_4_8, d_x_4_9, 0.;
            // clang-format on
        }
        return H_;
    };

    inline VMat V(const XVec&, const VVec&) const { return V_; };

    inline QMat Q(const double& dt) {
        double x;
        x            = 100;
        double t     = dt;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        if (side_flag_) {
            // clang-format off
            Q_ <<  q_x_x,  q_x_vx,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                  q_x_vx, q_vx_vx,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,  q_x_x,  q_x_vx, 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0., q_x_vx, q_vx_vx, 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 1., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   1e-10, 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0., 1.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0.,   0., 0.,  0., 3.0;
            // clang-format on
        } else {
            // clang-format off
            Q_ <<  q_x_x,  q_x_vx,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                  q_x_vx, q_vx_vx,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,  q_x_x,  q_x_vx, 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0., q_x_vx, q_vx_vx, 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 1., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   1e-10,  0.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0., 1.,  0.,
                      0.,      0.,     0.,      0., 0., 0., 0., 0.,   0.,  0., 3.0;
            // clang-format on
        }
        return Q_;
    };

    inline RMat R(const ZVec& z) {
        const double yaw_center_difference_ = std::abs(std::sin(z(0))) + 1.;
        if (second_armor_flag) {
            second_armor_flag = false;
            R_.diagonal() << 9999999999999., yaw_center_difference_ * r_theta_yaw_,
                yaw_center_difference_ * r_theta_pitch_, yaw_center_difference_ * r_theta_distance_;
        } else {
            R_.diagonal() << yaw_center_difference_ * r_theta_theta_,
                yaw_center_difference_ * r_theta_yaw_, yaw_center_difference_ * r_theta_pitch_,
                yaw_center_difference_ * r_theta_distance_;
        }
        return R_;
    };

    XVec f_ {};
    ZVec h_ {};
    AMat A_ {};
    WMat W_ { WMat::Identity() };
    HMat H_ {};
    VMat V_ { VMat::Identity() };
    QMat Q_ {};
    RMat R_ {};
    XVec normalized_x {};

    bool debug_ { true };
    double last_theta_ { std::numbers::pi };
    static constexpr double switch_angle_difference_ = std::numbers::pi * 70. / 180.;
    static constexpr double r_theta_theta_           = 0.1;
    static constexpr double r_theta_yaw_             = 1e-3;
    static constexpr double r_theta_pitch_           = 1e-3;
    static constexpr double r_theta_distance_        = 1.;

    bool side_flag_ { true };
    bool second_armor_flag { false };
};
}