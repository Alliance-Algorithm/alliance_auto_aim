#pragma once

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "interfaces/pnp_solver.hpp"
#include "interfaces/armor_in_camera.hpp"
#include "interfaces/armor_in_image.hpp"

namespace world_exe::alliance_auto_aim::tests {
    
    class MockPnpSolver : public world_exe::interfaces::IPnpSolver {
    public:
        MOCK_METHOD(std::shared_ptr<world_exe::interfaces::IArmorInCamera>, 
                   SolvePnp, 
                   (std::shared_ptr<world_exe::interfaces::IArmorInImage>), 
                   (override));
    };
    
    class MockArmorInCamera : public world_exe::interfaces::IArmorInCamera {
    public:
        MOCK_METHOD(const std::vector<world_exe::data::ArmorCameraSpacing>&, 
                   GetArmors, 
                   (const world_exe::enumeration::ArmorIdFlag& armor_id), 
                   (const, override));
        
        MOCK_METHOD(const world_exe::interfaces::ITimeStamped&, 
                   GetTimeStamped, 
                   (), 
                   (const, override));
    };
    
    class MockArmorInImage : public world_exe::interfaces::IArmorInImage {
    public:
        MOCK_METHOD(const std::vector<world_exe::data::ArmorImageSpacing>&, 
                   GetArmors, 
                   (const world_exe::enumeration::ArmorIdFlag& armor_id), 
                   (const, override));
        
        MOCK_METHOD(const world_exe::interfaces::ITimeStamped&, 
                   GetTimeStamped, 
                   (), 
                   (const, override));
    };
}