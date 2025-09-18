#pragma once

#include <functional>
#include <memory>

#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "tongji/predictor/time_stamp.hpp"
#include "util/extended_kalman_filter.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {

struct TargetStatus {
    bool jumped       = false;
    bool switched     = false;
    bool converged    = false;
    bool diverged     = false;
    bool lost         = false;
    bool reidentified = false;
    int last_id       = -1;
    int switch_count  = 0;
    int update_count  = 0;
    int lost_count    = 0;
};

class Target {
public:
    TargetStatus status_;

    Target(const Eigen::Vector3d& armor_xyz_in_world, const Eigen::Vector3d& armor_ypr_in_world,
        const enumeration::ArmorIdFlag& id, const std::time_t& t, const double& radius,
        const int& armor_num, const Eigen::VectorXd& P0_dig)
        : time_stamp_(t)
        , car_id_(id)
        , armor_num_(armor_num) {

        auto r = radius;

        // 旋转中心的坐标
        auto center_x = armor_xyz_in_world[0] + r * std::cos(armor_ypr_in_world[0]);
        auto center_y = armor_xyz_in_world[1] + r * std::sin(armor_ypr_in_world[0]);
        auto center_z = armor_xyz_in_world[2];

        // x vx y vy z vz a w r l h
        // a: angle
        // w: angular velocity
        // l: r2 - r1
        // h: z2 - z1
        Eigen::VectorXd x0 { { center_x, 0, center_y, 0, center_z, 0, armor_ypr_in_world[0], 0, r,
            0, 0 } }; // 初始化预测量
        Eigen::MatrixXd P0 = P0_dig.asDiagonal();

        // 防止夹角求和出现异常值
        auto x_add = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
            Eigen::VectorXd c = a + b;
            c[6]              = util::math::clamp_pm_pi(c[6]);
            return c;
        };

        ekf_ = util::ExtendedKalmanFilter(x0, P0, x_add); // 初始化滤波器（预测量、预测量协方差）
    }

    Eigen::VectorXd GetEkf_x() const { return ekf_.x; }

    void Predict(double dt) {
        // 防止夹角求和出现异常值
        auto f = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
            Eigen::VectorXd x_prior = this->F(dt) * x;
            x_prior[6]              = util::math::clamp_pm_pi(x_prior[6]);
            return x_prior;
        };

        // 前哨站转速特判
        if (this->Convergened() && this->car_id_ == enumeration::CarIDFlag::Outpost
            && std::abs(this->ekf_.x[7]) > 2)
            this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

        ekf_.Predict(this->F(dt), this->Q(dt), f);
    }

    std::time_t GetTimeStamp() const { return time_stamp_.GetTimeStamp(); }
    int GetArmorNum() const { return armor_num_; }
    enumeration::CarIDFlag GetId() const { return car_id_; }

    void Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data) {
        const auto& transform          = data->GetTransform();
        const auto& rotation_transform = Eigen::Quaterniond(transform.linear());
        const auto armors              = data->GetArmors()->GetArmors(car_id_);

        if (armors.empty()) return;
        for (auto armor : armors) {
            const auto& armor_in_world_position  = transform * armor.position;
            const auto& armor_in_world_oritaiton = rotation_transform * armor.orientation;
            this->Update(armor_in_world_position,
                util::math::quaternion_to_euler(armor_in_world_oritaiton, 2, 1, 0),
                util::math::xyz2ypd(armor_in_world_position));
        }
    }

    bool Convergened() {
        if (this->car_id_ != enumeration::CarIDFlag::Outpost && status_.update_count > 3
            && !this->Diverged()) {
            status_.converged = true;
        }

        // 前哨站特殊判断
        if (this->car_id_ == enumeration::CarIDFlag::Outpost && status_.update_count > 10
            && !this->Diverged()) {
            status_.converged = true;
        }

        return status_.converged;
    }

    bool Diverged() const {
        auto r_ok = ekf_.x[8] > 0.05 && ekf_.x[8] < 0.5;
        auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.05 && ekf_.x[8] + ekf_.x[9] < 0.5;

        if (r_ok && l_ok) return false;
        // util::logger::logger()->debug("[Target] r={:.3f}, l={:.3f}", ekf_.x[8], ekf_.x[9]);
        return true;
    }

private:
    void Update(const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world) {
        // 装甲板匹配
        int id = MatchArmor(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world);
        if (id != 0) status_.jumped = true;
        status_.switched = (id != status_.last_id);
        if (status_.switched) status_.switch_count++;

        status_.last_id = id;
        Update_ypda(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id);
        status_.update_count++;
    }

    void Update_ypda(const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world,
        int id) {
        // 观测jacobi
        Eigen::MatrixXd H = h_jacobian(ekf_.x, id);
        // Eigen::VectorXd R_dig{{4e-3, 4e-3, 1, 9e-2}};
        auto center_yaw  = std::atan2(armor_xyz_in_world[1], armor_xyz_in_world[0]);
        auto delta_angle = util::math::clamp_pm_pi(armor_ypr_in_world[0] - center_yaw);
        Eigen::VectorXd R_dig { { 4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
            log(std::abs(armor_ypd_in_world[2]) + 1) / 200 + 9e-2 } };

        // 测量过程噪声偏差的方差
        Eigen::MatrixXd R = R_dig.asDiagonal();

        // 定义非线性转换函数h: x -> z
        auto h = [&](const Eigen::VectorXd& x) -> Eigen::Vector4d {
            Eigen::VectorXd xyz = h_armor_xyz(x, id);
            Eigen::VectorXd ypd = util::math::xyz2ypd(xyz);
            auto angle          = util::math::clamp_pm_pi(x[6] + id * 2 * CV_PI / armor_num_);
            return { ypd[0], ypd[1], ypd[2], angle };
        };

        // 防止夹角求差出现异常值
        auto z_subtract = [](const Eigen::VectorXd& a,
                              const Eigen::VectorXd& b) -> Eigen::VectorXd {
            Eigen::VectorXd c = a - b;
            c[0]              = util::math::clamp_pm_pi(c[0]);
            c[1]              = util::math::clamp_pm_pi(c[1]);
            c[3]              = util::math::clamp_pm_pi(c[3]);
            return c;
        };

        const Eigen::VectorXd& ypd = armor_ypd_in_world;
        const Eigen::VectorXd& ypr = armor_ypr_in_world;
        Eigen::VectorXd z { { ypd[0], ypd[1], ypd[2], ypr[0] } }; // 获得观测量

        ekf_.update(z, H, R, h, z_subtract);
    }

    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd& x, int id) const {
        auto angle   = util::math::clamp_pm_pi(x[6] + id * 2 * CV_PI / armor_num_);
        auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

        auto r     = (use_l_h) ? x[8] + x[9] : x[8];
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

    int MatchArmor(const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world) {

        const auto& xyza_list = armor_xyza_list();
        std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;

        for (int i = 0; i < armor_num_; ++i) {
            xyza_i_list.emplace_back(xyza_list[i], i);
        }

        std::sort(xyza_i_list.begin(), xyza_i_list.end(), [](const auto& a, const auto& b) {
            auto ypd1 = util::math::xyz2ypd(a.first.head(3));
            auto ypd2 = util::math::xyz2ypd(b.first.head(3));
            return ypd1[2] < ypd2[2];
        });

        int best_id      = 0;
        double min_error = 1e10;

        for (int i = 0; i < std::min(3, armor_num_); ++i) {
            const auto& xyza = xyza_i_list[i].first;
            auto ypd         = util::math::xyz2ypd(xyza.head(3));
            double error     = std::abs(util::math::clamp_pm_pi(armor_ypr_in_world[0] - xyza[3]))
                + std::abs(util::math::clamp_pm_pi(armor_ypd_in_world[0] - ypd[0]));

            if (error < min_error) {
                min_error = error;
                best_id   = xyza_i_list[i].second;
            }
        }

        return best_id;
    }

    std::vector<Eigen::Vector4d> armor_xyza_list() const {
        std::vector<Eigen::Vector4d> _armor_xyza_list;

        for (int i = 0; i < armor_num_; i++) {
            auto angle          = util::math::clamp_pm_pi(ekf_.x[6] + i * 2 * CV_PI / armor_num_);
            Eigen::Vector3d xyz = h_armor_xyz(ekf_.x, i);
            _armor_xyza_list.push_back({ xyz[0], xyz[1], xyz[2], angle });
        }
        return _armor_xyza_list;
    }

    // 计算出装甲板中心的坐标（考虑长短轴）
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd& x, int id) const {
        auto angle   = util::math::clamp_pm_pi(x[6] + id * 2 * CV_PI / armor_num_);
        auto use_l_h = (armor_num_ == 4) && (id == 1 || id == 3);

        auto r       = (use_l_h) ? x[8] + x[9] : x[8];
        auto armor_x = x[0] - r * std::cos(angle);
        auto armor_y = x[2] - r * std::sin(angle);
        auto armor_z = (use_l_h) ? x[4] + x[10] : x[4];

        return { armor_x, armor_y, armor_z };
    }

    auto F(double dt) -> Eigen::MatrixXd {
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

auto Q(double dt)->Eigen::MatrixXd{
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

    TimeStamp time_stamp_;
    util::ExtendedKalmanFilter ekf_;
    enumeration::CarIDFlag car_id_;
    int armor_num_;
};

}