#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/opencv.hpp>

#include "../src/util/profile/profile.hpp"
#include "armor_pnp_solver.hpp"

using namespace world_exe::pnpsolver;

class StaticImpl {
public:
    static std::optional<const world_exe::data::ArmorCameraSpacing> Solve(
        const world_exe::data::ArmorImageSpacing& armors) {

        cv::Mat rvec, tvec;
        auto& objectPoints = armors.isLargeArmor ? LargeArmorObjectPoints : NormalArmorObjectPoints;
        if (cv::solvePnP(objectPoints, armors.image_points,
                world_exe::util::Profile::get_intrinsic_parameters(),
                world_exe::util::Profile::get_distortion_parameters(), rvec, tvec, false,
                cv::SOLVEPNP_IPPE)) {

            Eigen::Vector3d position = { tvec.at<double>(2), -tvec.at<double>(0),
                -tvec.at<double>(1) };
            position                 = position / 1000.0;
            if (position.norm() > MaxArmorDistance) {
                return {};
            }

            Eigen::Vector3d rvec_eigen  = { rvec.at<double>(2), -rvec.at<double>(0),
                 -rvec.at<double>(1) };
            Eigen::Quaterniond rotation = Eigen::Quaterniond { Eigen::AngleAxisd {
                rvec_eigen.norm(), rvec_eigen.normalized() } };

            return { { armors.id, position, rotation } };
        }
        return {};
    }

private:
    inline constexpr static const double MaxArmorDistance = 15.0;

    inline constexpr static const double NormalArmorWidth = 134, NormalArmorHeight = 56,
                                         LargerArmorWidth = 230, LargerArmorHeight = 56;
    inline static const std::vector<cv::Point3d> LargeArmorObjectPoints = {
        cv::Point3d(-0.5 * LargerArmorWidth, 0.5 * LargerArmorHeight, 0.0f),
        cv::Point3d(-0.5 * LargerArmorWidth, -0.5 * NormalArmorHeight, 0.0f)
    };

    inline static const std::vector<cv::Point3d> NormalArmorObjectPoints = {
        cv::Point3d(0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
        cv::Point3d(0.5 * NormalArmorWidth, 0.5 * NormalArmorHeight, 0.0f)
    };
};

const world_exe::interfaces::IArmorInCamera& ArmorIPPEPnPSolver::SolvePnp(
    const world_exe::interfaces::IArmorInImage& armors) {
    for (int i = 0; i < static_cast<int>(world_exe::enumeration::ArmorIdFlag::Count); i++) {
        armors_[i].clear();
        for (const auto& armor : armors.GetArmors(static_cast<enumeration::ArmorIdFlag>(i))) {
            const auto& armor_in_camera = StaticImpl::Solve(armor);
            if (armor_in_camera.has_value())
                armors_[i].emplace_back(std::move(armor_in_camera.value()));
        }
    }
    return *this;
}

const world_exe::interfaces::ITimeStamped& ArmorIPPEPnPSolver::GetTimeStamped() const {
    return *this;
}

const std::time_t& ArmorIPPEPnPSolver::GetTimeStamp() const { return time_point_; }

const std::vector<world_exe::data::ArmorCameraSpacing>& ArmorIPPEPnPSolver::GetArmors(
    const world_exe::enumeration::ArmorIdFlag& armor_id) const {
    return armors_[static_cast<int>(armor_id)];
}