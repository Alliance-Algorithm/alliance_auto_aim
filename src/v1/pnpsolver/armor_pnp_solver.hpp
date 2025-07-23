#pragma once
#include <opencv2/opencv.hpp>

#include "interfaces/pnp_solver.hpp"

namespace world_exe::v1::pnpsolver {

class ArmorIPPEPnPSolver final : public interfaces::IPnpSolver,
                                 public interfaces::IArmorInCamera,
                                 public interfaces::ITimeStamped {
public:
    void set_time_point(const std::time_t& time_point);

    const interfaces::IArmorInCamera& SolvePnp(const interfaces::IArmorInImage& armor) override;
    const ITimeStamped& GetTimeStamped() const override;
    const std::time_t& GetTimeStamp() const override;
    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override;

private:
    std::time_t time_point_;
    std::vector<data::ArmorCameraSpacing>
        armors_[static_cast<int>(enumeration::ArmorIdFlag::Count)];
};
}