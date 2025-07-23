#pragma once

namespace world_exe::v1 {
class SystemV1 {
public:
    static void Build();
    static void Build_1();

    class SystemV1Impl;

private:
    static SystemV1Impl* instance_;

    SystemV1()                = default;
    SystemV1(const SystemV1&) = delete;
    ~SystemV1()               = delete;
};
}
