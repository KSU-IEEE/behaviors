#ifndef RETURN_TO_START_H
#define RETURN_TO_START_H

#include <base_behavior.h>
/**************************************************
pac-man-bot::return_to_start

This is the second to last state in the SM, it will
return the bot to the home position

End statement: sent_ && done_moving_ 
sent_ is set true when initialy sending pose
done_moving_ is set when the vehicle stops moving
==================================================
CONNECTIONS
subscribers: finishedMove

publishers: goTo
**************************************************/
// ros includes
#include <behaviors/coordinate.h>

namespace behaviors {
namespace pac_man_behs {
class return_to_start : public base_behavior {
public:
    return_to_start();
    ~return_to_start();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;

    // subscribers and cb
    ros::Subscriber sub_finishedMove;
    void finishedMove_cb(const std_msgs::Bool::ConstPtr& yes) {done_moving_ = yes->data;}

    // publisher 
    ros::Publisher pub_goTo, armReset;
private:
    // return statements
    bool done_moving_ = false;
    bool sent_ = false;

    // constants
    const std::pair<float, float> HOME_LOC = {42.5, 1.5};

};  // return_to_start
} // pac_man_behs
} // behaviors


#endif  // RETURN_TO_START_H