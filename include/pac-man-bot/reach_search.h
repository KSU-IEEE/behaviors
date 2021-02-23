#ifndef REACH_SEARCH_H
#define REACH_SEARCH_H

#include <pac-man-bot/ground_search.h>
/**************************************************
pac-man-bot::reach_search

This class is used to search over the walls in 
the pac-man-bot arena. To do this, we will take in
the distance sent from the arm to monitor when an 
block passes from underneath it to grab.

This also inherits from ground_search because there
were a lot of similarities between the two. Bc of 
this, it has the same fsm as class.

search_done_
Set when the search state in the fsm finishes
==================================================
CONNECTIONS
subscribers: 
    ALL_SUBS_FROM_GROUND_SEARCH
    armDistance

publishers: publishes to topics
**************************************************/
namespace behaviors {
namespace pac_man_behs {
class reach_search : public ground_search {
public:
    reach_search();
    ~reach_search();

    // override - from base_behavior
    // bool control_loop() override; // not needed, same fsm as ground_search
    void set_params() override;
    void nodelet_init() override;

    // override - from ground_search
    bool moveToStart() override;  // returns true when completed move
    bool search() override;       // returns true when finding a block
    bool grabBlock() override;    // returns true when returned back to staritng position
    bool next() override;
    bool check() override;

    // internal
    bool initialArmPos();
    bool doneWithSearch();
    void moveArm();
    void moveBot();

    // subs and cbs
    ros::Subscriber sub_armDistance;
    void armDistance_cb(const std_msgs::Float64::ConstPtr& dist);

private:
    // ros vars
    float arm_dist_;
    std::pair<float, float> arm_loc_;

    // internal
    float arm_to_ground_;
    int num_with_wall_ = 0;
    bool initial_box_search_ = true;
    bool search_outward_ = true;  // true when arm moving away false when arm moving inward
    bool move_ = false;
    std::pair<float, float> previour_arm_loc_ ;
    int curr_count_col_ = 0;

    // constants
    const int TARGET_HEADING = 90;
    const int ROWS_TO_SEARCH_AT_WALL = 7;
    const int BOT_FROM_LEFT_WALL = 4;
    const int BOT_FROM_RIGHT_WALL = 92;
    const int LENGTH_OF_ABC = 7.5;
    const int LENGTH_OF_DE = 4.5;
    const std::pair<float, float> LOCATION_A = {4, 11.5};
    const std::pair<float, float> LOCATION_B = {39.5, 11.5};
    const std::pair<float, float> LOCATION_C = {115, 11.5};
    const std::pair<float, float> LOCATION_D = {4, 27.5};
    const std::pair<float, float> LOCATION_E = {69, 27.5};

}; // reach_search
}  // pac_man_behs
}  // behaviors


#endif  // REACH_SEARCH_H