#pragma once

#include "enum/car_id.hpp"
#include "util/math.hpp"

namespace world_exe::tongji::fire_control {

class AimPointChooser {
public:
    AimPointChooser(
        const double& comming_angle = 60 / 57.3, const double& leaving_angle = 20 / 57.3)
        : comming_angle_(comming_angle)
        , leaving_angle_(leaving_angle) { }

    std::pair<bool, Eigen::Vector4d> ChooseAimArmor(const Eigen::Vector<double, 11>& ekf_x,
        const std::vector<Eigen::Vector4d>& xyza_list, const enumeration::CarIDFlag& single_id) {
        const auto armor_num = xyza_list.size();
        int chosen_id        = -1;

        // 整车旋转中心的球坐标yaw
        const auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

        // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
        std::vector<double> delta_angle_list;
        for (int i = 0; i < armor_num; i++) {
            auto delta_angle = util::math::clamp_pm_pi(xyza_list[i][3] - center_yaw);
            delta_angle_list.emplace_back(delta_angle);
        }

        // 不考虑小陀螺
        if (std::abs(ekf_x[8]) <= 2 && single_id != enumeration::CarIDFlag::Outpost) {
            // 选择在可射击范围内的装甲板
            std::vector<int> id_list;
            for (int i = 0; i < armor_num; i++) {
                if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
                id_list.push_back(i);
            }

            if (id_list.size() == 1) {
                chosen_id = id_list[0];
                lock_id_  = -1;
            } else if (id_list.size() > 1) {
                const int id0 = id_list[0], id1 = id_list[1];
                // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
                if (lock_id_ != id0 && lock_id_ != id1)
                    lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1]))
                        ? id0
                        : id1;
                chosen_id = lock_id_;
            } else {
                chosen_id = -1;
            }
        } else {
            // 小陀螺
            double coming_angle =
                (single_id == enumeration::CarIDFlag::Outpost) ? 70 / 57.3 : comming_angle_;
            double leaving_angle =
                (single_id == enumeration::CarIDFlag::Outpost) ? 30 / 57.3 : leaving_angle_;

            // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
            for (int i = 0; i < armor_num; i++) {
                if (std::abs(delta_angle_list[i]) > coming_angle) continue;
                if ((ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle)
                    || (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle)) {
                    chosen_id = i;
                    break;
                }
            }
        }

        if (chosen_id == -1) {
            return { false, xyza_list[0] };
        }

        return {
            true,
            xyza_list[chosen_id],
        };
    }

private:
    double comming_angle_ = 60 / 57.3;
    double leaving_angle_ = 20 / 57.3;
    int lock_id_          = -1;
};

}
