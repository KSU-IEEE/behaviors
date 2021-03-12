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
    bool move() override;
    bool search() override;       // returns true when finding a block
    bool grabBlock() override;    // returns true when returned back to staritng position

    // utils
    std::pair<float, float> calcBounds();
    std::pair<float, float> getInitial();
    

private:

    // constants
    std::pair<float, float> LOCATION_A = {4, 11.5};
    std::pair<float, float> LOCATION_B = {39.5, 11.5};
    std::pair<float, float> LOCATION_C = {115, 11.5};
    std::pair<float, float> LOCATION_D = {4, 27.5};
    std::pair<float, float> LOCATION_E = {69, 27.5};

}; // reach_search
}  // pac_man_behs
}  // behaviors


#endif  // REACH_SEARCH_H