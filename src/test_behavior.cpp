#include "behaviors/test_behavior.h"
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(behaviors::test_behavior, nodelet::Nodelet);
namespace behaviors {
test_behavior::test_behavior() : base_behavior(){
    name_ = "tester";
}
test_behavior::~test_behavior(){}

bool test_behavior::control_loop(){
    NODELET_INFO("TREVOR IS IN CONTROL ON THIS CRAZY RIDE");
}
void test_behavior::set_params(){

}
void test_behavior::nodelet_init(){

}
}