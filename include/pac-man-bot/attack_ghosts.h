#ifndef ATTACK_GHOSTS_H
#define ATTACK_GHOSTS_H

#include <base_behavior.h>
/**************************************************
pac-man-bot::attack_ghosts

The attach ghosts state attacks ghosts(duh). It 
will listen to the ghost location topic and 
when it is transitioned into, it will navigate to
both ghosts that are sent to it at the beginning.
This also has a timer in it to keep track of the 
power period. If the timer ends, it gives up
control

End statement: 
    ghosts_killed_ || timer_
    ghosts_killed_ is a boolean that gets set true
        when both ghosts have been bumped
    timer_ is a boolean that gets set to true when
        the timer hits 0
==================================================
CONNECTIONS
subscribers: 
    ghostLocation
    heading
    finishedMove
    arm/grabBlock

publishers: 
    goTo
    setHeading
    arm/Done

**************************************************/
// ros includes
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <behaviors/coordinate.h>

// other includes
#include <chrono>
#include <utility>

namespace behaviors {
namespace pac_man_behs {

enum ag_fsm{move, set_heading, grab};

class attack_ghosts : public base_behavior {
public:
    attack_ghosts();
    ~attack_ghosts();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;

    // ros subscribers and cbs
    ros::Subscriber sub_ghostLoc, sub_finishedMove, 
                    sub_heading, sub_armDone;
    void ghostLoc_cb(const std_msgs::Int8::ConstPtr& loc);
    void heading_cb(const std_msgs::Float64::ConstPtr& degree);
    void finishedMove_cb(const std_msgs::Bool::ConstPtr& yes);
    void armDone_cb(const std_msgs::Bool::ConstPtr& yes);

    // ros pubs
    ros::Publisher pub_goTo, pub_setHeading, pub_armGrabBlock;

    // internal 
    bool stopwatch(int seconds);
    bool move();
    bool grab();
private:
    // ros vars
    float heading_;
    bool finished_move_ = true;

    // end conditions
    bool ghosts_killed_, timer_;

    // function of arm parameters
    float distancePickUp_;

    // timer componenets
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    int last_msg_ = -1;
    bool initialRun_ = true;

    // fsm
    ag_fsm fsm = ag_fsm::move;
    bool sent_loc_ = false;
    bool grabbing = false;
    bool start_grabbing = false;
    int target_head_;
    bool arm_done_ = false;

    // ghosts bools
    bool ghost1_ = false;
    bool ghost2_ = false;
    bool ghost3_ = false;
    bool ghost4_ = false;
    bool ghost5_ = false;

    // constants
    const std::pair<float, float> GHOST_LOC1 = {34.5, 24};
    const std::pair<float, float> GHOST_LOC2 = {64.5, 24};
    const std::pair<float, float> GHOST_LOC3 = {29.5, 10.5};
    const std::pair<float, float> GHOST_LOC4 = {64.5, 10.5};
    const std::pair<float, float> GHOST_LOC5 = {39.5, 16};

    
};  // attack_ghosts
} // pac_man_behs
} // behaviors
#endif  // ATTACK_GHOSTS_H