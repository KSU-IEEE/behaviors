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
    /arm/distance

publishers:
    ghostLocation
    /arm/scan
**************************************************/

// ros includes
#include <std_msgs/Float64.h>
#include <behaviors/distances.h>
#include <behaviors/coordinate.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h> 
#include <behaviors/polar_coordinate.h>

// random includes
#include <vector>
#include <utility> // utility
#include <cmath>

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
    ros::Subscriber sub_armDistance;

    void armDistance_cb(const std_msgs::Float64::ConstPtr& dist);

    ros::Publisher pub_scan, pub_ghostLoc;

    void moveArm();
    bool calcExistance(float dist);
    void sendGhostLoc();
    
private:
    // end conditions
    bool ghost1_ = false;
    bool ghost2_ = false;

    // params
    float ghost_dist_tol_;
    float sensorDist, baseHeight, armLength;

    // list of posible ghost locations
    // id is index
    // first is x, second is y
    std::vector<std::pair<float, float>> locs_;
    int curr_target = 0;
    
    bool sent_loc = false;
    float r_sent;
    float armBaseLength;

    // CONSTANTS
    float START_X, START_Y;

};  // check_for_ghosts
} // pac_man_behs
} // behaviors
#endif  // CHECK_FOR_GHOSTS_H