#include "armor_image_detail.hpp"

namespace world_exe::interfaces::detail {
ArmorInImage::GetArmor(const enumeration::ArmorIdFlag& armor_id) 
{
    for(const auto& armor : armors_)
    {
        if(armor.id == armor_id)
        {
            return armor;
        }
    }
    else
    {
        return 0;
    }
}
}