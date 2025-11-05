#pragma  once
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <data/armor_gimbal_control_spacing.hpp>
#include <data/time_stamped.hpp>
#include <enum/car_id.hpp>
#include <interfaces/armor_in_gimbal_control.hpp>
#include <chrono>
namespace world_exe::tests::mock{
    
class MockArmorInWorld final : public world_exe::interfaces::IArmorInGimbalControl{

    public:

    MockArmorInWorld(double angular_speed = 1) 
        : time(std::chrono::steady_clock::now().time_since_epoch())
        , armor(){
            armor.emplace_back(
                enumeration::CarIDFlag::InfantryIII,
                Eigen::Vector3d{1,0,1}, 
                Eigen::Quaterniond{
                Eigen::AngleAxisd(time.to_seconds() * angular_speed, Eigen::Vector3d::UnitY()).toRotationMatrix()}
            );
    }

    /// 获取时间戳，标志其内容装甲板的准确时间点
    const data::TimeStamp& GetTimeStamp() const{
        return time;
    }

    /// 获取某个车辆ID的装甲板集合
    const std::vector<data::ArmorGimbalControlSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const
    {
        return armor;
    }

    private:
    data::TimeStamp                                 time;
    std::vector<data::ArmorGimbalControlSpacing>    armor;
};
}