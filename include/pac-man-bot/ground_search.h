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
    heading
    distances
    position

publishers: 
    goTo
    setHeading

*NOTE* will need to add arm topics here later
**************************************************/
// ros includes
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <behaviors/distances.h>
#include <behaviors/coordinate.h>

// random includes
#include <utility> // std::pair

namespace behaviors {
namespace pac_man_behs {
// define state machine
enum fsm {moveToStart, search, grabBlock};

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
    ros::Subscriber sub_finishedMove, sub_heading, sub_distances,
                    sub_posiiton;
    void finishedMove_cb(const std_msgs::Bool::ConstPtr& yes);
    void heading_cb(const std_msgs::Float64::ConstPtr& degree);
    void distances_cb(const behaviors::distances::ConstPtr& data);
    void pos_cb(const behaviors::coordinate::ConstPtr& data);


    // publishers 
    ros::Publisher pub_goTo, pub_setHeading;

    // fsm functions
    bool moveToStart();  // returns true when completed move
    bool search();       // returns true when finding a block
    bool grabBlock();    // returns true when returned back to staritng position

    // internal funcs
    bool doneMoving();

    // search functions
    bool checkHeading();
    bool next();
    bool check();
private:
// I want to reuse this for reach_search, so movign everything to protected

protected:
    // end condition 
    bool search_done_ = false;

    // call back vars
    bool done_moving_ = false;
    int heading_;
    float curr_x_, curr_y_;
    float front_dist_, back_dist_, left_dist_, right_dist_;

    // state machine
    fsm state; // -- see below

    // internal vars
    float return_to_x_, return_to_y_;
    float target_x_, target_y_;
    float sent_loc_ = false;
    bool waiting_for_head_ = false;
    bool ready_to_check_ = true;
    bool checked_ahead_ = false;

    // constants -- in form x,y
    const std::pair<float, float> LOCATION_F = {5.5, 35};
    const std::pair<float, float> LOCATION_H = {95, 35};
};  // ground_search

} // pac_man_behs
} // behaviors


#endif  // GROUND_SEARCH_H