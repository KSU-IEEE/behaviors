#include "test/test_behavior.h"
PLUGINLIB_EXPORT_CLASS(behaviors::tests::test_behavior, nodelet::Nodelet);
namespace behaviors {
namespace tests{
test_behavior::test_behavior(){}
test_behavior::~test_behavior(){}

bool test_behavior::control_loop(){
    ROS_INFO("TREVOR IS IN CONTROL ON THIS CRAZY RIDE");
}
void test_behavior::set_params(){

}
void test_behavior::nodelet_init(){

}
}
}