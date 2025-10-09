#include "Pnpsolver.cpp" 
#include <gmock/gmock.h>
#include <gtest/gtest.h>
using namespace world_exe::interfaces;
using namespace world_exe::data;
using namespace world_exe::enumeration;

using ::testing::Return;
using ::testing::_;

TEST_F(PnpsolverTest,AbilityTest)
{
    RunTest();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}