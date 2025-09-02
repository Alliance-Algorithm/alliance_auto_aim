#pragma once

#include <memory>
namespace world_exe::tongji::identifier {

class Classifier final {
public:
    Classifier();
    ~Classifier();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

}