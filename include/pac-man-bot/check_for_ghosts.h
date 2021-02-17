#ifndef CHECK_FOR_GHOSTS_H
#define CHECK_FOR_GHOSTS_H

#include <base_behavior.h>

/**************************************************
pac-man-bot::check_for_ghosts:

This is the second state within the pac-man-bot sm.
It checks where the ghosts are publishes that out to
all the states that need it.

End statement: 
    ghost1_ && ghost2_
    both ghosts have been found
==================================================
CONNECTIONS
subscribers: 
    distances
    heading
    finishedMove

publishers:
    ghostLocation
    goTo
**************************************************/

// ros includes
#include <std_msgs/Float64.h>
#include <behaviors/distances.h>
#include <behaviors/coordinate.h>
#include <std_msgs/Int8.h>

// random includes
#include <vector>
#include <utility> // utility

namespace behaviors {
namespace pac_man_behs {
class check_for_ghosts : public base_behavior {
public:
    check_for_ghosts();
    ~check_for_ghosts();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;

    // subs and cbs
    ros::Subscriber subDistances, subHeading,
                    subFinishedMove;
    void distances_cb(const behaviors::distances::ConstPtr& data);
    void heading_cb(const std_msgs::Float64::ConstPtr& degree)
        {heading_ = int(degree->data) % 360;} // keep it simple
    void finishedMove_cb(const std_msgs::Bool::ConstPtr& yes)
        {done_moving_ = yes->data;}

    ros::Publisher pubGhostLoc, pubGoTo;

    // class specific
    bool checkGhosts();
    void sendMoveCmd();
    void sendGhostLoc();
    
private:
    // end conditions
    bool ghost1_ = false;
    bool ghost2_ = false;

    // distances
    float front_dist_, back_dist_, left_dist_,
          right_dist_;
    int heading_;
    bool done_moving_ = true;

    // params
    float ghost_dist_tol_;

    // list of posible ghost locations
    // id is index
    // first is x, second is y
    std::vector<std::pair<float, float>> locs_;
    int curr_target = -1;

};  // check_for_ghosts
} // pac_man_behs
} // behaviors
#endif  // CHECK_FOR_GHOSTS_H