#ifndef TEST_BEHAVIOR_H
#define TEST_BEHAVIOR_H 

#include "base_behavior.h"

// ros message includes
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

namespace behaviors {
class test_behavior : public base_behavior {
public:
    test_behavior();
    ~test_behavior();

    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;

    void test1_cb(const std_msgs::String::ConstPtr& msg);
    void printnum_cb(const std_msgs::Empty::ConstPtr& msg);
private: 
    std::string message;
    int num;

    ros::Subscriber sub_test2;
    ros::Publisher pub_message;
    ros::Subscriber sub_printNum;
};
} // namespace behaviors

#endif // TEST_BEHAVIOR_H