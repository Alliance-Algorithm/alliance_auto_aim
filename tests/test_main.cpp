#include "test_main.hpp"

using namespace world_exe::alliance_auto_aim::tests;
using namespace world_exe::interfaces;
using namespace world_exe::data;
using namespace world_exe::enumeration;

using ::testing::Return;
using ::testing::_;

TEST(DataFlowTest, Datain_out)
{
    MockPnpSolver Mock_solver;
    auto mock_armor_in_image = std::make_shared<MockArmorInImage>();
    auto mock_armor_in_camera = std::make_shared<MockArmorInCamera>();
    std::shared_ptr<IArmorInImage> armor_in_image = mock_armor_in_image;
    std::shared_ptr<IArmorInCamera> armor_in_camera = mock_armor_in_camera;

    EXPECT_CALL(Mock_solver,SolvePnp(armor_in_image)).WillOnce(Return(armor_in_camera));

    auto result = Mock_solver.SolvePnp(armor_in_image);
    EXPECT_EQ(result, armor_in_camera);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleMock(&argc, argv);
    return RUN_ALL_TESTS();
}