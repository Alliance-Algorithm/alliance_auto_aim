#pragma once
#include "interfaces/armor_in_image.hpp"

namespace world_exe::interfaces::detail{
    class ArmorInImage : public world_exe::interfaces::IArmorInImage {
    public:
        ArmorInImage(const std::vector<world_exe::data::ArmorImageSpacing>& armors)
            : armors_(armors) {}
        virtual ~ArmorInImage() = default;
        COMBINE_TIME_STAMPED;
        const std::vector<world_exe::data::ArmorImageSpacing> armors_;
        const std::vector<world_exe::data::ArmorImageSpacing>& GetArmors(const enumeration::ArmorIdFlag& armor_id) const override;
    private:

    };
}