#include "Pnpsolver.cpp" 
#include "v1/pnpsolver/armor_pnp_solver.hpp"
#include <gmock/gmock.h>
#include <gtest/gtest.h>

using namespace world_exe::v1::pnpsolver;
using world_exe::parameters::Robomaster;

using ::testing::Return;
using ::testing::_;

TEST_P(PnpsolverTest,AbilityTest)
{
    ArmorIPPEPnPSolver pnp_solver_test_v1(Robomaster::NormalArmorObjectPointsOpencv,Robomaster::LargeArmorObjectPointsOpencv);
    RunTest();

}
INSTANTIATE_TEST_SUITE_P(PnpsolverTest, PnpsolverTest, ::testing::Values(
    new ArmorIPPEPnPSolver(Robomaster::NormalArmorObjectPointsOpencv,Robomaster::LargeArmorObjectPointsOpencv)
    ));

    // INSTANTIATE__SUITE_P()
    // 测试套件
    // 测试类名
    // 测试类的参数传入->GetParam()
int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}