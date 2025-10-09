#include <gtest/gtest.h>
#include <interfaces/pnp_solver.hpp>
#include "mocks/MockArmor2D.hpp"
#include "v1/pnpsolver/armor_pnp_solver.hpp"
#include "parameters/rm_parameters.hpp"
#include "mocks/MockHikCameraProfile.hpp"

using world_exe::v1::pnpsolver::ArmorIPPEPnPSolver;
using world_exe::tests::mock::MockArmorInImage;
using world_exe::parameters::Robomaster;

void RunableTest(world_exe::interfaces::IPnpSolver& pnp_solver)
{
    auto mockarmor = MockArmorInImage::createMockArmorInImage();
    pnp_solver.SolvePnp(mockarmor);
}

class PnpsolverTest : public ::testing::Test {
protected:
    ArmorIPPEPnPSolver pnp_solver;
    PnpsolverTest(): pnp_solver( Robomaster::NormalArmorObjectPointsOpencv,Robomaster::LargeArmorObjectPointsOpencv){}

    void RunTest() {
        RunableTest(pnp_solver);
    }
};