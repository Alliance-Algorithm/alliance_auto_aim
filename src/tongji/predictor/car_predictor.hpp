#pragma once

#include <memory>
#include <opencv2/core/cvdef.h>

#include "data/armor_gimbal_control_spacing.hpp"
#include "enum/armor_id.hpp"
#include "enum/car_id.hpp"
#include "interfaces/predictor_update_package.hpp"
#include "tongji/predictor/time_stamp.hpp"
#include "util/extended_kalman_filter.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::predictor {

struct PredictorStatus {
    bool jumped       = false;
    bool switched     = false;
    bool converged    = false;
    bool diverged     = false;
    bool lost         = false;
    bool reidentified = false;
    int switch_count  = 0;
    int update_count  = 0;
    int lost_count    = 0;
};

class CarPredictor {
public:
    CarPredictor() = default;
    explicit CarPredictor(const enumeration::ArmorIdFlag armor_id,
        const Eigen::Vector3d& armor_xyz_in_world, const Eigen::Vector3d& armor_ypr_in_world,
        std::time_t time_stamp, double radius, int armor_num, Eigen::VectorXd P0_dig)
        : armor_id_(armor_id)
        , last_id(0)
        , radius_(radius)
        , armor_num_(armor_num)
        , last_predict_time_stamp_(time_stamp) {
        const Eigen::VectorXd& xyz = armor_xyz_in_world;
        const Eigen::VectorXd& ypr = armor_ypr_in_world;

        // 旋转中心的坐标
        auto center_x = xyz[0] + radius * std::cos(ypr[0]);
        auto center_y = xyz[1] + radius * std::sin(ypr[0]);
        auto center_z = xyz[2];

        // x vx y vy z vz a w r l h
        // a: angle
        // w: angular velocity
        // l: r2 - r1
        // h: z2 - z1
        Eigen::VectorXd x0 { { center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, radius, 0,
            0 } }; // 初始化预测量
        P0_dig_            = P0_dig;
        Eigen::MatrixXd P0 = P0_dig.asDiagonal();

        // 防止夹角求和出现异常值
        x_add_ = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
            Eigen::VectorXd c = a + b;
            c[6]              = util::math::clamp_pm_pi(c[6]);
            return c;
        };

        ekf_ = util::ExtendedKalmanFilter(x0, P0, x_add_); // 初始化滤波器（预测量、预测量协方差）
    }

    void SetEkf(const util::ExtendedKalmanFilter& ekf) { ekf_ = ekf; }

    std::vector<data::ArmorGimbalControlSpacing> GetPredictedArmors(
        enumeration::ArmorIdFlag armor_id, const TimeStamp time_stamp) {
        PredictTo(time_stamp.GetTimeStamp());
        UpdateStatus();

        // 装甲板匹配
        int id;
        auto min_angle_error                          = 1e10;
        const std::vector<Eigen::Vector4d>& xyza_list = armor_xyza_list();

        std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
        for (int i = 0; i < armor_num_; i++) {
            xyza_i_list.push_back({ xyza_list[i], i });
        }

        std::sort(xyza_i_list.begin(), xyza_i_list.end(),
            [](const std::pair<Eigen::Vector4d, int>& a, const std::pair<Eigen::Vector4d, int>& b) {
                Eigen::Vector3d ypd1 = util::math::xyz2ypd(a.first.head(3));
                Eigen::Vector3d ypd2 = util::math::xyz2ypd(b.first.head(3));
                return ypd1[2] < ypd2[2];
            });

        std::vector<data::ArmorGimbalControlSpacing> armors;
        for (int i = 0; i < armor_num_; i++) {
            xyza_i_list.push_back({ xyza_list[i], i });
            const auto pos     = xyza_i_list[i].first.head<3>();
            const auto yaw     = util::math::clamp_pm_pi(ekf_.x[6]);
            const double pitch = 15. / 180. * std::numbers::pi;
            armors.push_back(data::ArmorGimbalControlSpacing {
                armor_id_, pos, util::math::euler_to_quaternion(yaw, pitch, 0.) });
        }
        return armors;
    }

    void Update(std::shared_ptr<interfaces::IPreDictorUpdatePackage> data) {
        const auto& transform          = data->GetTransform();
        const auto& rotation_transform = Eigen::Quaterniond(transform.linear());
        const auto armors              = data->GetArmors()->GetArmors(armor_id_);

        if (armors.empty()) return;
        for (auto armor : armors) {
            const auto& armor_in_world_position  = transform * armor.position;
            const auto& armor_in_world_oritaiton = rotation_transform * armor.orientation;
            this->update(armor_in_world_position,
                util::math::quaternion_to_euler(armor_in_world_oritaiton, 2, 1, 0),
                util::math::xyz2ypd(armor_in_world_position));
        }
    }

    const PredictorStatus& GetStatus() const { return status_; }

    void PredictTo(const TimeStamp& now) {
        if (last_predict_time_stamp_.GetTimeStamp() == 0) {
            last_predict_time_stamp_ = now;
            return;
        }
        const double& dt         = now.SecondsSince(last_predict_time_stamp_);
        last_predict_time_stamp_ = now;

        if (dt <= 0.0 || dt > 1.0) {
            // util::logger::logger()->warn(
            //     "[CarPredictor] Abnormal dt: {:.3f}, skipping prediction", dt);
            return;
        }

        predict(dt);
    }

    void Reset(const Eigen::Vector3d& xyz, const Eigen::Vector3d& ypr, std::time_t time_stamp) {
        last_predict_time_stamp_ = TimeStamp::FromRaw(time_stamp);
        status_                  = PredictorStatus {}; // 清空状态计数器

        // 重新构造滤波器状态
        auto center_x = xyz[0] + radius_ * std::cos(ypr[0]);
        auto center_y = xyz[1] + radius_ * std::sin(ypr[0]);
        auto center_z = xyz[2];

        Eigen::VectorXd x0 { { center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, radius_, 0, 0 } };
        Eigen::MatrixXd P0 = P0_dig_.asDiagonal();

        ekf_ = util::ExtendedKalmanFilter(x0, P0_dig_, x_add_);
    }

    void TickStatus() {
        static int last_update_count = -1;
        if (status_.update_count == last_update_count) {
            status_.lost_count++;
        } else {
            status_.lost_count = 0;
        }
        last_update_count    = status_.update_count;
        status_.lost         = (status_.lost_count > 30 || IsDiverged());
        status_.reidentified = (!status_.lost && IsConverged() && status_.update_count > 3);
    }

    bool IsLocked() const { return status_.converged && !status_.diverged; }
    bool IsLost() const { return status_.lost; }
    bool IsReidentified() const { return status_.reidentified; }

private:
    bool IsConverged() const { return status_.converged; }
    bool IsDiverged() const { return status_.diverged; }

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

    auto F(double dt) const -> Eigen::MatrixXd {
        // 状态转移矩阵
        // clang-format off
        return Eigen::MatrixXd {
           {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
           {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
           {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
           {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
           {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
           {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
           {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
           {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
           {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
           {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
           {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
        };
        // clang-format on
    }

    auto Q(double dt) const -> Eigen::MatrixXd {
        // Piecewise White Noise Model
        // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
        double v1, v2;
        if (armor_id_ == enumeration::CarIDFlag::Outpost) {
            v1 = 10;  // 前哨站加速度方差
            v2 = 0.1; // 前哨站角加速度方差
        } else {
            v1 = 100; // 加速度方差
            v2 = 400; // 角加速度方差
        }
        auto a = dt * dt * dt * dt / 4;
        auto b = dt * dt * dt / 2;
        auto c = dt * dt;
        // 预测过程噪声偏差的方差
        // clang-format off
        return  Eigen::MatrixXd {
          {a * v1, b * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
          {b * v1, c * v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
          {     0,      0, a * v1, b * v1,      0,      0,      0,      0, 0, 0, 0},
          {     0,      0, b * v1, c * v1,      0,      0,      0,      0, 0, 0, 0},
          {     0,      0,      0,      0, a * v1, b * v1,      0,      0, 0, 0, 0},
          {     0,      0,      0,      0, b * v1, c * v1,      0,      0, 0, 0, 0},
          {     0,      0,      0,      0,      0,      0, a * v2, b * v2, 0, 0, 0},
          {     0,      0,      0,      0,      0,      0, b * v2, c * v2, 0, 0, 0},
          {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
          {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
         {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
        };
        // clang-format on
    }

    void predict(const double& dt) {
        // 防止夹角求和出现异常值
        auto f = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
            Eigen::VectorXd x_prior = this->F(dt) * x;
            x_prior[6]              = util::math::clamp_pm_pi(x_prior[6]);
            return x_prior;
        };

        // 前哨站转速特判
        if (IsConverged() && this->armor_id_ == enumeration::CarIDFlag::Outpost
            && std::abs(this->ekf_.x[7]) > 2)
            this->ekf_.x[7] = this->ekf_.x[7] > 0 ? 2.51 : -2.51;

        ekf_.predict(this->F(dt), this->Q(dt), f);
    }

    void UpdateStatus() {
        // 发散判断
        double r         = ekf_.x[8];
        double l         = ekf_.x[9];
        status_.diverged = !(r > 0.05 && r < 0.5 && r + l > 0.05 && r + l < 0.5);

        // 收敛判断
        if (armor_id_ == enumeration::CarIDFlag::Outpost) {
            status_.converged = (status_.update_count > 10 && !status_.diverged);
        } else {
            status_.converged = (status_.update_count > 3 && !status_.diverged);
        }
    }

    void update(const Eigen::Vector3d& armor_xyz_in_world,
        const Eigen::Vector3d& armor_ypr_in_world, const Eigen::Vector3d& armor_ypd_in_world) {
        // 装甲板匹配
        int id = MatchArmor(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world);

        if (id != 0) status_.jumped = true;
        status_.switched = (id != last_id);
        if (status_.switched) status_.switch_count++;

        last_id = id;
        update_ypda(armor_xyz_in_world, armor_ypr_in_world, armor_ypd_in_world, id);
        status_.update_count++;
        UpdateStatus();
    }

    void update_ypda(const Eigen::Vector3d& armor_xyz_in_world,
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

    Eigen::VectorXd ekf_x() const { return ekf_.x; }

    const util::ExtendedKalmanFilter& ekf() const { return ekf_; }

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

    int armor_num_;
    double radius_;
    PredictorStatus status_;

    enumeration::ArmorIdFlag armor_id_;
    int last_id = -1; // debug only

    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> x_add_;
    Eigen::VectorXd P0_dig_;
    util::ExtendedKalmanFilter ekf_;
    TimeStamp last_predict_time_stamp_;
};

}