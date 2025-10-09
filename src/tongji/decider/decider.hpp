

#pragma once

#include <memory>
#include <vector>

#include "enum/armor_id.hpp"
#include "tongji/decider/armor_info.hpp"

namespace world_exe::tongji::decider {

enum PriorityMode { MODE_ONE = 1, MODE_TWO };

class Decider {
public:
    Decider(PriorityMode mode = PriorityMode::MODE_ONE);
    ~Decider();

    void SetInvincibleArmors(const enumeration::ArmorIdFlag& armors);
    void SetPriority(std::vector<ArmorInfo>& detected_result) const;
    enumeration::ArmorIdFlag GetSortedArmor(std::vector<ArmorInfo>& armors) const;
    bool ArmorFilter(std::vector<ArmorInfo>& armors);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}