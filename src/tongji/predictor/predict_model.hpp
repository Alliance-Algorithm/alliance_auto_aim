#pragma once

#include <functional>

#include <Eigen/Dense>
#include <opencv2/core/cvdef.h>

#include "enum/car_id.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {

class PredictModel {
public:
    PredictModel(const enumeration::CarIDFlag& car_id)
        : car_id_(car_id) {

        bool is_balance = (car_id == enumeration::CarIDFlag::InfantryIII
            || car_id == enumeration::CarIDFlag::InfantryIV
            || car_id == enumeration::CarIDFlag::InfantryV);

        P0_dig_.resize(11);
        if (is_balance) {
            P0_dig_ << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
        } else if (car_id == enumeration::CarIDFlag::Outpost) {
            P0_dig_ << 1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0;
        } else if (car_id == enumeration::CarIDFlag::Base) {
            P0_dig_ << 1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0;
        } else {
            P0_dig_ << 1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1;
        }

        if (car_id == enumeration::CarIDFlag::Outpost) {
            radius_ = 0.2765;
        } else if (car_id == enumeration::CarIDFlag::Base) {
            radius_ = 0.3205;
        } else {
            radius_ = 0.2;
        }

        if (is_balance) {
            armor_num_ = 2;
        } else if (car_id == enumeration::CarIDFlag::Outpost
            | car_id == enumeration::CarIDFlag::Base) {
            armor_num_ = 3;
        } else {
            armor_num_ = 4;
        }
    }

    auto GetP0Dig() const { return P0_dig_; }
    auto GetRadius() const { return radius_; }
    auto GetID() const { return car_id_; }
    int GetArmorNum() const { return armor_num_; }

    // 防止夹角求和出现异常值
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add =
        [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            Eigen::VectorXd c = a + b;
            c(6)              = util::math::clamp_pm_pi(c(6));
            return c;
        };

    // 防止夹角求和出现异常值
    std::function<Eigen::VectorXd(const Eigen::VectorXd& x, const double& dt)> f =
        [=, this](const Eigen::VectorXd& x, const double& dt) -> Eigen::VectorXd {
        Eigen::VectorXd x_prior = A(dt) * x;
        x_prior(6)              = util::math::clamp_pm_pi(x_prior(6));
        return x_prior;
    };

    int MatchArmor(const Eigen::VectorXd& x, const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world,
        const Eigen::Vector3d& armor_ypd_in_world) const {

        const auto& xyza_list = GetArmorXYZAList(x);
        std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;

        for (int i = 0; i < armor_num_; ++i) {
            xyza_i_list.emplace_back(xyza_list[i], i);
        }

        std::sort(xyza_i_list.begin(), xyza_i_list.end(), [](const auto& a, const auto& b) {
            auto ypd1 = util::math::xyz2ypd(a.first.head(3));
            auto ypd2 = util::math::xyz2ypd(b.first.head(3));
            return ypd1(2) < ypd2(2);
        });

        int best_id      = 0;
        double min_error = 1e10;

        for (int i = 0; i < std::min(3, armor_num_); ++i) {
            const auto& xyza = xyza_i_list[i].first;
            auto ypd         = util::math::xyz2ypd(xyza.head(3));
            double error     = std::abs(util::math::clamp_pm_pi(armor_ypr_in_world(0) - xyza(3)))
                + std::abs(util::math::clamp_pm_pi(armor_ypd_in_world(0) - ypd(0)));

            if (error < min_error) {
                min_error = error;
                best_id   = xyza_i_list[i].second;
            }
        }
        return best_id;
    }

    // 计算出装甲板中心的坐标（考虑长短轴）
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd& x, int id) const {
        auto angle   = util::math::clamp_pm_pi(x(6) + id * 2 * CV_PI / armor_num_);
        auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

        auto r       = (use_l_h) ? x(8) + x(9) : x(8);
        auto armor_x = x(0) - r * std::cos(angle);
        auto armor_y = x(2) - r * std::sin(angle);
        auto armor_z = (use_l_h) ? x(4) + x(10) : x(4);

        return { armor_x, armor_y, armor_z };
    }

    Eigen::MatrixXd H(const Eigen::VectorXd& x, int id) const {
        auto angle   = util::math::clamp_pm_pi(x(6) + id * 2 * CV_PI / armor_num_);
        auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

        auto r     = (use_l_h) ? x(8) + x(9) : x(8);
        auto dx_da = r * std::sin(angle);
        auto dy_da = -r * std::cos(angle);

        auto dx_dr = -std::cos(angle);
        auto dy_dr = -std::sin(angle);
        auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
        auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;

        auto dz_dh = (use_l_h) ? 1.0 : 0.0;

        // clang-format off
  Eigen::MatrixXd H_armor_xyza{
    {1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0},
    {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0},
    {0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh},
    {0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0}
  };
        // clang-format on

        Eigen::VectorXd armor_xyz   = h_armor_xyz(x, id);
        Eigen::MatrixXd H_armor_ypd = util::math::xyz2ypd_jacobian(armor_xyz);
        // clang-format off
        Eigen::MatrixXd H_armor_ypda{
          {H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
          {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
          {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
          {                0,                 0,                 0, 1}
        };
        // clang-format on

        return H_armor_ypda * H_armor_xyza;
    }

    Eigen::MatrixXd R(const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world,
        int id) {
        // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
        auto center_yaw  = std::atan2(armor_xyz_in_world(1), armor_xyz_in_world(0));
        auto delta_angle = util::math::clamp_pm_pi(armor_ypr_in_world(0) - center_yaw);
        Eigen::VectorXd R_dig(4);
        R_dig << 4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
            log(std::abs(armor_ypd_in_world(2)) + 1) / 200 + 9e-2;

        // 测量过程噪声偏差的方差
        return R_dig.asDiagonal();
    }

    Eigen::MatrixXd A(double dt) const {
        // 状态转移矩阵
        // clang-format off
    return (Eigen::MatrixXd(11, 11) <<
        1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,
        0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1).finished();
    }
     //clang-format on

 Eigen::MatrixXd Q(double dt) const { 
        // Piecewise White Noise Model
        // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
    double v1, v2;
    if (car_id_ == enumeration::CarIDFlag::Outpost) {
      v1 = 10;   // 前哨站加速度方差
      v2 = 0.1;  // 前哨站角加速度方差
    } else {
      v1 = 100;  // 加速度方差
      v2 = 400;  // 角加速度方差
    }
    auto a = dt * dt * dt * dt / 4;
    auto b = dt * dt * dt / 2;
    auto c = dt * dt;
      
    // 预测过程噪声偏差的方差
    // clang-format off
    return (Eigen::MatrixXd(11, 11) <<
    a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,
    b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,
         0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0,
         0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0,
         0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0,
         0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0,
         0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0,
         0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0,
         0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,
         0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,
         0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0).finished();
        // clang-format on
    }

    std::function<Eigen::VectorXd(const Eigen::VectorXd&, int)> h = [this](const Eigen::VectorXd& x,
                                                                        int id) {
        Eigen::Vector3d xyz = this->h_armor_xyz(x, id);
        Eigen::Vector3d ypd = util::math::xyz2ypd(xyz);
        double angle        = util::math::clamp_pm_pi(x(6) + id * 2 * CV_PI / this->armor_num_);
        return Eigen::Vector4d { ypd(0), ypd(1), ypd(2), angle };
    };

    // 防止夹角求差出现异常值
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> z_subtract =
        [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            Eigen::VectorXd c = a - b;
            c(0)              = util::math::clamp_pm_pi(c(0));
            c(1)              = util::math::clamp_pm_pi(c(1));
            c(3)              = util::math::clamp_pm_pi(c(3));
            return c;
        };

    std::vector<Eigen::Vector4d> GetArmorXYZAList(const Eigen::VectorXd& ekf_x) const {
        std::vector<Eigen::Vector4d> _armor_xyza_list;

        for (int i = 0; i < armor_num_; i++) {
            auto angle          = util::math::clamp_pm_pi(ekf_x(6) + i * 2 * CV_PI / armor_num_);
            Eigen::Vector3d xyz = h_armor_xyz(ekf_x, i);
            _armor_xyza_list.push_back({ xyz(0), xyz(1), xyz(2), angle });
        }
        return _armor_xyza_list;
    }

private:
    int armor_num_;
    enumeration::CarIDFlag car_id_;

    Eigen::VectorXd P0_dig_;
    double radius_;
};

}