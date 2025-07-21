#pragma once

namespace world_exe::v1 {
class SystemV1 {
public:
    static void Build();

    class SystemV1ImplBase;

private:
    static SystemV1ImplBase* instance_;

    SystemV1()                = default;
    SystemV1(const SystemV1&) = delete;
    ~SystemV1()               = delete;
};
}