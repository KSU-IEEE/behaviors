#ifndef BASE_BEHAVIOR_H
#define BASE_BEHAVIOR_H
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string.h>

using namespace std;
namespace behaviors {
class base_behavior {
public:
    // init and deinit
    base_behavior();
    base_behavior(string node_name);
    ~base_behavior();

    // virtual functions
    virtual bool control_loop();
    virtual void set_params();
    virtual void init();
    
    void set_nh(ros::NodeHandle nh);
    ros::NodeHandle nh();
    string node_name();
    void set_node_name(string name);
    bool in_control();

    // ros callbacks
    void activate_cb(const std_msgs::Bool::ConstPtr& activate);
private:
    string name_ = "base";
    bool in_control_ = true;
protected:
    ros::NodeHandle nh_;   
}; // class base_behavior
} // namespace behaviors

#endif // BASE_BEHAVIOR_H