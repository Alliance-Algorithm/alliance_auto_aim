#pragma once

#include <memory>

#include "interfaces/target_predictor.hpp"

namespace world_exe::tongji::predictor {
class TargetPredict final : public interfaces::ITargetPredictor {
public:
  
private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
}