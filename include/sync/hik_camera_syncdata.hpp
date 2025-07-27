#pragma once
#include "opencv2/core/mat.hpp"

#include "data/sync_data.hpp"
#include "sync/sync_data.hpp"

namespace world_exe::sync {

struct HikCameraSyncData final
    : public world_exe::core::SyncData<cv::Mat, world_exe::data::CameraGimbalMuzzleSyncData> {
public:
    std::tuple<cv::Mat&, world_exe::data::CameraGimbalMuzzleSyncData&> load() override {
        return { mat_, data_ };
    };
    void store(cv::Mat arg1, world_exe::data::CameraGimbalMuzzleSyncData arg2) override {
        arg1.copyTo(mat_);
        std::memcpy((void*)&data_, (void*)&arg2, sizeof(data_));
    };
    explicit HikCameraSyncData(const cv::Mat& mat)
        : mat_ { mat.clone() }
        , data_ {} { }

private:
    cv::Mat mat_;
    world_exe::data::CameraGimbalMuzzleSyncData data_;
};
} // namespace world_exe::sync