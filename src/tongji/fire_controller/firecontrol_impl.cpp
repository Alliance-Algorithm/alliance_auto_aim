
#include "./firecontrol.hpp"
#include "tongji/fire_controller/tracker.hpp"

using namespace world_exe::tongji::fire_controller;
class FireControl::Impl {
    // 后续转依赖注入
public:
    explicit Impl(DefaultTracker tracker) { }

    // impl interface here
};
