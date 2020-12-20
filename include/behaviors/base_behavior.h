#ifndef BASE_BEHAVIOR_H
#define BASE_BEHAVIOR_H
#include <ros/ros.h>
#include <string.h>

using namespace std;
namespace behaviors {
class base_behavior {
public:
    // virtual functions
    virtual bool control_loop();
    virtual void set_params();

    void set_nh(ros::NodeHandle nh);
    ros::NodeHandle nh();
    string node_name();
    bool in_control();
private:
    string name_ = "base";
    /*
    TODO:
    Need to set basic subscriber to see if in control
    */
    bool in_control_ = true;
protected:
    ros::NodeHandle nh_;   
}; // class base_behavior
} // namespace behaviors

#endif // BASE_BEHAVIOR_H