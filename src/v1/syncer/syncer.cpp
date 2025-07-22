
#include "syncer.hpp"
#include "update_package.hpp"
#include <chrono>

namespace world_exe::v1::sync {
class Syncer::Impl {
public:
    Impl() { sync_data_buffer_.reserve(50); }

    std::tuple<const interfaces::IPreDictorUpdatePackage&, bool> await(double t_second = 2) {
        bool ready { false };
        const auto end_time_point =
            std::chrono::steady_clock::now() + std::chrono::duration<double>(t_second);
        while (std::chrono::steady_clock::now() <= end_time_point) {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto data_num = sync_data_buffer_.size();
            for (size_t i = data_num - 1; i >= 0; i--) {
                if (std::abs((camera_capture_end_time_stamp_
                                 - sync_data_buffer_[i].camera_capture_begin_time_stamp)
                        - delay_)
                    <= threshold) {
                    ready = true;
                    predict_update_package_.SetTimeStamp(
                        sync_data_buffer_[i].camera_capture_begin_time_stamp);
                    predict_update_package_.SetTransform(sync_data_buffer_[i].camera_to_gimbal);
                }
            }
            sync_data_buffer_.clear();
            if (ready) break;
        }

        return { predict_update_package_, ready };
    }

    void setTimeStamp(const std::time_t& time) { camera_capture_end_time_stamp_ = time; }

    void SetMainData(const interfaces::IArmorInCamera& armor_in_camera) {
        predict_update_package_.SetArmor(armor_in_camera);
    }

    void LoadCallback(const data::CameraGimbalMuzzleSyncData& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        sync_data_buffer_.emplace_back(data);
    }

private:
    PredictorUpdatePackage predict_update_package_;
    std::time_t camera_capture_end_time_stamp_ { 0 };

    static constexpr std::time_t delay_    = 1e6;
    static constexpr std::time_t threshold = 1e6;

    std::mutex mutex_;
    std::vector<data::CameraGimbalMuzzleSyncData> sync_data_buffer_;
};

Syncer::Syncer()
    : pimpl_(std::make_unique<Impl>()) { };

std::tuple<const interfaces::IPreDictorUpdatePackage&, bool> Syncer::await(double t_second) {
    return pimpl_->await(t_second);
}

void Syncer::SetCameraCaptureEndTimeStamp(const std::time_t& time) {
    return pimpl_->setTimeStamp(time);
}

void Syncer::SetMainData(const interfaces::IArmorInCamera& armor_in_camera) {
    return pimpl_->SetMainData(armor_in_camera);
}

void Syncer::LoadCallback(const data::CameraGimbalMuzzleSyncData& data) const {
    return pimpl_->LoadCallback(data);
}

Syncer::~Syncer() = default;
}
