#pragma once

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>
#include <numeric>

namespace world_exe::tongji::predictor {
template <int xn, int zn> //
class ExtendedKalmanFilter {
public:
    using XVec = Eigen::Matrix<double, xn, 1>;
    using ZVec = Eigen::Matrix<double, zn, 1>;
    using AMat = Eigen::Matrix<double, xn, xn>;
    using PMat = Eigen::Matrix<double, xn, xn>;
    using PDig = Eigen::Matrix<double, xn, 1>;
    using RMat = Eigen::Matrix<double, zn, zn>;
    using RDig = Eigen::Matrix<double, zn, 1>;
    using QMat = Eigen::Matrix<double, xn, xn>;
    using HMat = Eigen::Matrix<double, zn, xn>;

    XVec x;
    PMat P;

    ExtendedKalmanFilter() = default;

    ExtendedKalmanFilter(
        const XVec& x0, const PMat& P0,
        std::function<XVec(const XVec&, const XVec&)> x_add = [](const XVec& a, const XVec& b) {
            return a + b;
        }) {
        data["residual_yaw"]        = 0.0;
        data["residual_pitch"]      = 0.0;
        data["residual_distance"]   = 0.0;
        data["residual_angle"]      = 0.0;
        data["nis"]                 = 0.0;
        data["nees"]                = 0.0;
        data["nis_fail"]            = 0.0;
        data["nees_fail"]           = 0.0;
        data["recent_nis_failures"] = 0.0;
    }

    XVec Update(
        const double& dt, const AMat& A, const QMat& Q,
        std::function<XVec(const XVec&, const double&)> f, const ZVec& z, const HMat& H,
        const RMat& R, std::function<ZVec(const XVec&)> h,
        std::function<ZVec(const ZVec&, const ZVec&)> z_subtract =
            [](const ZVec& a, const ZVec& b) { return a - b; }) {

        auto x_n = f(x, dt);

        auto P_n = A * P * A.transpose() + Q;

        auto residual = z_subtract(z, h(x_n));

        auto S = H * P * H.transpose() + R;

        auto K = P_n * H.transpose() * S.inverse();

        x = x_add(x, K * residual);

        // Stable Compution of the Posterior Covariance
        // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
        P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

        /// 卡方检验
        // 新增检验
        double nis = residual.transpose() * S.inverse() * residual;

        // 卡方检验阈值（自由度=4，取置信水平95%）
        constexpr double nis_threshold = 0.711;

        if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
        total_count_++;
        last_nis = nis;

        recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

        if (recent_nis_failures.size() > window_size) {
            recent_nis_failures.pop_front();
        }

        int recent_failures =
            std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
        double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

        data["residual_yaw"]        = residual[0];
        data["residual_pitch"]      = residual[1];
        data["residual_distance"]   = residual[2];
        data["residual_angle"]      = residual[3];
        data["nis"]                 = nis;
        data["recent_nis_failures"] = recent_rate;

        return x;
    }

    std::map<std::string, double> data; // 卡方检验数据
    std::deque<int> recent_nis_failures { 0 };
    size_t window_size = 100;
    double last_nis;

private:
    Eigen::Matrix<double, xn, xn> I;
    std::function<XVec(const XVec&, const XVec&)> x_add;

    int nees_count_  = 0;
    int nis_count_   = 0;
    int total_count_ = 0;
};

} // namespace tools
