#ifndef GROUND_SEARCH_H
#define GROUND_SEARCH_H
#include <base_behavior.h>
/**************************************************
project::state:
pac-man-bot::ground_search

ground search will perform the search in the two 
boxes f and h. This will have a small internal 
state machine:
- move to start
    - moves to the starting point for the search
- search
    - slowly moves down the wall
    - exists when at the bottom
grab block
    - if a block is found during the search, then
        grab it
    - returns to last point in search

End statement: 
search_done_
Set when the search state in the fsm finishes
==================================================
CONNECTIONS
subscribers:
    finishedMove
    position
    /arm/done
    /arm/distance

publishers: 
    goTo
    /arm/scan
    /arm/grabBlock

*NOTE* will need to add arm topics here later
**************************************************/
// ros includes
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <behaviors/distances.h>
#include <behaviors/coordinate.h>
#include <behaviors/polar_coordinate.h>

// random includes
#include <utility> // std::pair

namespace behaviors {
namespace pac_man_behs {
// define state machine
enum fsm {moving, search, grabBlock};

class ground_search : public base_behavior {
public:
    ground_search();
    ground_search(std::string overrideName);
    ~ground_search();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;

    // subsribers and cbs
    ros::Subscriber sub_finishedMove, sub_posiiton, sub_armDist, sub_armDon;
    void finishedMove_cb(const std_msgs::Bool::ConstPtr& yes);
    void pos_cb(const behaviors::coordinate::ConstPtr& data);
    void armDist_cb(const std_msgs::Float64::ConstPtr& val);
    void armDone_cb(const std_msgs::Bool::ConstPtr& yes);

    // publishers 
    ros::Publisher pub_goTo, pub_armScan, pub_armGrab;

    // fsm functions
    virtual bool move();
    virtual bool search();       // returns true when finding a block
    virtual bool grabBlock();    // returns true when returned back to staritng position
    virtual bool checkTransition();

    // internal funcs
    bool doneMoving();
    behaviors::polar_coordinate calc_message();
    bool calcExistance(float dist);
private:
// I want to reuse this for reach_search, so movign everything to protected

protected:
    // end condition 
    bool search_done_ = false;

    // call back vars
    bool done_moving_ = false;
    bool arm_done_moving_ = false;
    float curr_x_, curr_y_;
    float botWidth = 3;

    // state machine
    fsm state; // -- see below

    // internal vars
    float target_x_, target_y_;
    float sent_loc_ = false;
    bool waiting_for_head_ = false;
    bool ready_to_check_ = true;
    bool checked_ahead_ = false;
    bool setInitialPoint = false;
    bool blockChecked = false;
    bool move_;
    bool arm_moving_ = false;

    // arm vars
    float armLength, armBaseLength, r_sent;
    float baseHeight, ghost_dist_tol_;

    std::pair<float, float> currentCheck;
    std::pair<float, float> initialCheck;
    bool left = false; // when true, the x values decrease by 1, otherwise they incse by one

    // two chunks of each block
    bool doFirstBlock = true; // when true, do first search patter
                              // when false, do second

    // constants -- in form x,y
    std::pair<float, float> LOCATION_FS = {4, 34.5}; // starting f
    std::pair<float, float> LOCATION_F =  {4, 34.5}; // second f loc
    std::pair<float, float> LOCATION_HS = {78.5, 34.5};  // starting h
    std::pair<float, float> LOCATION_H = {78.5, 34.5};   // second H
};  // ground_search

} // pac_man_behs
} // behaviors


#endif  // GROUND_SEARCH_H