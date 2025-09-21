#pragma once
#include "data/armor_in_image.hpp"

namespace world_exe::interfaces::detail{
    class ArmorInImage : public world_exe::interfaces::IArmorInImage {
    public:
        ArmorInImage(const std::vector<world_exe::data::ArmorImageSpacing>& armors)
            : armors_(armors) {}
        virtual ~ArmorInImage() = default;
        const std::vector<world_exe::data::ArmorImageSpacing>& GetArmors(const enumeration::ArmorFlag& armor_id) const override;
    private:
        std::vector<world_exe::data::ArmorImageSpacing> armors_;
    };
}