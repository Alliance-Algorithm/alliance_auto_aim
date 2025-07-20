#pragma once

namespace world_exe::v1 {
class SystemV1 {
public:
    static void Build();

private:
    class SystemV1Impl;
    static SystemV1Impl* instance_;

    SystemV1()                = default;
    SystemV1(const SystemV1&) = delete;
    ~SystemV1()               = delete;
};
}