#ifndef BASE_BEHAVIOR_H
#define BASE_BEHAVIOR_H
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>
#include <behaviors/target.h>
#include <behaviors/completed.h>

#include <string.h>

#include<boost/thread.hpp>

// use this header in inhertied class
/**************************************************
project::state:

A short summary

End statement: what ends this state
==================================================
CONNECTIONS
subscribers: subscibes to topics

publishers: publishes to topics
**************************************************/

using namespace std;
namespace behaviors {
class base_behavior : public nodelet::Nodelet {
public:
    // used to run
    void onInit() override;

    // init and deinit
    base_behavior();
    base_behavior(string node_name);
    ~base_behavior();

    // virtual functions
    virtual bool control_loop();
    virtual void set_params();
    virtual void nodelet_init();
    void parent_control_loop();

    void set_nh(ros::NodeHandle nh);
    ros::NodeHandle nh();
    string node_name();
    void set_node_name(string name);
    bool in_control();
    void set_goal(char val){goal_ = val;}
    char get_goal() {return goal_;}


    void print_msg(std::string msg);

    // ros callbacks
    void activate_cb(const behaviors::target::ConstPtr& activate);
private:
    // control cariables
    bool in_control_ = false;
    char goal_;

    boost::shared_ptr<boost::thread> deviceThread_;
    // subscribers and publishers
    ros::Subscriber sm_sub;
    ros::Publisher sm_pub;

protected:
    string name_ = "base";
    ros::NodeHandle nh_; 


}; // class base_behavior
} // namespace behaviors

#endif // BASE_BEHAVIOR_H