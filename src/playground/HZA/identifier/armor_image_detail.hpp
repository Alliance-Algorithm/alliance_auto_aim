#pragma once
#include "interfaces/armor_in_image.hpp"
#include "interfaces/time_stamped.hpp"
#include "time_stamped_detail.hpp"

namespace world_exe::interfaces::detail{
    class ArmorInImage : public world_exe::interfaces::IArmorInImage {
    public:
        ArmorInImage(const std::vector<world_exe::data::ArmorImageSpacing>& armors_)
            : armors_(armors_) {}
        virtual ~ArmorInImage() = default;
        const ITimeStamped& GetTimeStamped() const override;
        const std::vector<world_exe::data::ArmorImageSpacing> armors_;
        const std::vector<world_exe::data::ArmorImageSpacing>& GetArmors(const enumeration::ArmorIdFlag& armor_id) const override;
    private:
        TimeStamped time_stamped_;

    };
}