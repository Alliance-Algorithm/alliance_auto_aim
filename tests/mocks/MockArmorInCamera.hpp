#pragma  once
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <cmath>
#include <data/armor_camera_spacing.hpp>
#include <data/armor_gimbal_control_spacing.hpp>
#include <data/time_stamped.hpp>
#include <enum/car_id.hpp>
#include <interfaces/armor_in_camera.hpp>
#include <interfaces/armor_in_gimbal_control.hpp>
#include <chrono>
namespace world_exe::tests::mock{
    
class MockArmorInCamera final : public world_exe::interfaces::IArmorInCamera{

    public:
    const enumeration::CarIDFlag armorid = enumeration::CarIDFlag::InfantryIII;
    MockArmorInCamera(double angular_speed = 1,double speed = 1) 
        : time(std::chrono::steady_clock::now().time_since_epoch())
        , armor()
        , none(){
            armor.emplace_back(
                armorid,
                Eigen::Vector3d{3,0.2 * sin(time.to_seconds() * speed),0.2 * cos(2 * time.to_seconds() * speed)}, 
                Eigen::Quaterniond{
                Eigen::AngleAxisd(sin(time.to_seconds() * angular_speed) , Eigen::Vector3d::UnitZ()).toRotationMatrix()}
            );
    }

    /// 获取时间戳，标志其内容装甲板的准确时间点
    const data::TimeStamp& GetTimeStamp() const{
        return time;
    }

    /// 获取某个车辆ID的装甲板集合
    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const
    {
        if(armor_id == armorid)
            return armor;
        return none;
    }

    private:
    data::TimeStamp                                 time;
    std::vector<data::ArmorCameraSpacing>           armor;
    std::vector<data::ArmorCameraSpacing>           none;
};
}