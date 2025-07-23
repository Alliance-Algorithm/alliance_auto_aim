#include "update_package.hpp"
#include <memory>

namespace world_exe::v1::sync {
class PredictorUpdatePackage::Impl : public interfaces::IArmorInCamera, interfaces::ITimeStamped {
public:
    Impl()
        : armor_interface_(*this) { }

    const ITimeStamped& GetTimeStamped() const override { return *this; };
    const std::time_t& GetTimeStamp() const override { return time_stamp_; }

    const std::vector<data::ArmorCameraSpacing>& GetArmors(
        const enumeration::ArmorIdFlag& armor_id) const override {
        if (&armor_interface_ == this) {
            static const std::vector<data::ArmorCameraSpacing> empty;
            return empty;
        }
        return armor_interface_.GetArmors(armor_id);
    }

    const Eigen::Affine3d& GetTransform() const { return transform_; }

    void SetArmor(const interfaces::IArmorInCamera& armor_in_camera) {
        armor_interface_ = armor_in_camera;
    }
    void SetTransform(const Eigen::Affine3d& transform) { transform_ = transform; }
    void SetTimeStamp(const std::time_t& time_stamp) { time_stamp_ = time_stamp; }

private:
    std::time_t time_stamp_;
    interfaces::IArmorInCamera& armor_interface_;
    Eigen::Affine3d transform_;
};

PredictorUpdatePackage::PredictorUpdatePackage()
    : pimpl_(std::make_unique<Impl>()) { };

const interfaces::ITimeStamped& PredictorUpdatePackage::GetTimeStamped() const {
    return pimpl_->GetTimeStamped();
}
PredictorUpdatePackage::PredictorUpdatePackage(const PredictorUpdatePackage& in)
    : pimpl_(std::make_unique<Impl>(*(in.pimpl_))) {

    };

const interfaces::IArmorInCamera& PredictorUpdatePackage::GetArmors() const { return *pimpl_; }

const Eigen::Affine3d& PredictorUpdatePackage::GetTransform() const {
    return pimpl_->GetTransform();
}

void PredictorUpdatePackage::SetArmor(const interfaces::IArmorInCamera& armor_in_camera) {
    return pimpl_->SetArmor(armor_in_camera);
}
void PredictorUpdatePackage::SetTransform(const Eigen::Affine3d& transform) {
    pimpl_->SetTransform(transform);
}
void PredictorUpdatePackage::SetTimeStamp(const std::time_t& time_stamp) {
    pimpl_->SetTimeStamp(time_stamp);
}

PredictorUpdatePackage::~PredictorUpdatePackage() { };
}