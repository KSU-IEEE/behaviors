#include <pac-man-bot/reach_search.h>

// export
PLUGINLIB_EXPORT_CLASS(behaviors::pac_man_behs::reach_search, nodelet::Nodelet);

namespace behaviors {
namespace pac_man_behs{
// inits and deinits
/**************************************************************/
reach_search::reach_search()
    : ground_search("reach_search"){}
reach_search::~reach_search(){}

// ros callbacks
/**************************************************************/


// utilities
/**************************************************************/
std::pair<float, float> reach_search::calcBounds() {
    // first handle A,B,C bc they have the same Ybounds
    std::pair<float,float> returner;
    if(get_goal() == 'A' || get_goal() == 'B' || get_goal() == 'C') {
        returner.second = 47;  // y bound
        // x bounds
        if(get_goal() == 'A')
            returner.first = left ? .5 : 17;
        else if (get_goal() == 'B')
            returner.first = left ? 42 : 52;
        else 
            returner.first = left ? 79 : 95;
    } else {
        returner.second = 26;  // y bound
        // x bounds
        if(get_goal() == 'D')
            returner.first = left ? .5 : 23;
        else 
            returner.first = left ? 70 : 92;
    }

    return returner;
}

std::pair<float, float> reach_search::getInitial() {
    float x, y;
    switch(get_goal()) {
        case 'A':
            x = .5;
            y = 40;
            break;
        case 'B':
            x = 41.5;
            y = 40;
            break;
        case 'C':
            x = 66;
            y = 40;
            break;
        case 'D':
            x = .5;
            y = 21.5;
        case 'E':
            x = 69; // nice
            y = 21.5;
    }

    return std::make_pair(x, y);
}

// override - from ground_search
/**************************************************************/
bool reach_search::move() {
    if (!sent_loc_ && !done_moving_) {
        behaviors::coordinate msg;
        switch(get_goal()) {
            case('A'):
                msg.X = LOCATION_A.first;
                msg.Y = LOCATION_A.second;
                break;
            case('B'):
                msg.X = LOCATION_B.first;
                msg.Y = LOCATION_B.second;
                break;
            case('C'):
                msg.X = LOCATION_C.first;
                msg.Y = LOCATION_C.second;
                break;
            case('D'):
                msg.X = LOCATION_D.first;
                msg.Y = LOCATION_D.second;
                break;
            case('E'):
                msg.X = LOCATION_E.first;
                msg.Y = LOCATION_E.second;
                break;                                                                
        }
        pub_goTo.publish(msg);

        sent_loc_ = true;
        done_moving_ = false;
        search_done_ = false;
        setInitialPoint = true;
    }
}

bool reach_search::search() {
if(!arm_moving_){
    if(setInitialPoint) {
        initialCheck = getInitial();
        currentCheck = initialCheck;
    }
    std::pair<float, float> bounds = calcBounds();

    if (left) {
        currentCheck.first--;
        if (currentCheck.first <= bounds.first) {
            currentCheck.first++;
            currentCheck.second++;
        }
    } else {
        currentCheck.first++;
        if(currentCheck.first >= bounds.first) {
            currentCheck.first--;
            currentCheck.second++;
        }
    }

    // check if y is in bounds, and quit if not
    if (currentCheck.second > bounds.second) {
        search_done_ = true;
        return true;
    }

    // get angle
    behaviors::polar_coordinate msg = calc_message();
    pub_armScan.publish(msg);

    arm_moving_=true;
}
}

bool reach_search::grabBlock() {
    // first move
    if(!sent_loc_ && !done_moving_) {
        // directly below the block, an arm distance from it
        behaviors::coordinate msg;
        msg.X = currentCheck.first;
        msg.Y = currentCheck.second - armLength;
        pub_goTo.publish(msg);

        sent_loc_ = true;
        done_moving_ = false;
    } else if(!arm_moving_){
        // send arm command
        behaviors::polar_coordinate temp = calc_message();
        std_msgs::Float64 msg;
        msg.data = temp.theta;

        pub_armGrab.publish(msg);
        arm_moving_ = true;
        sent_loc_ = false;
    }
}


// override functions - from base_behavior
/**************************************************************/
void reach_search::set_params() {
    ground_search::set_params();
}

void reach_search::nodelet_init() {
    ground_search::nodelet_init();
    // update locs
    LOCATION_A = {9, 39.5 - armLength};
    LOCATION_B = {43.5, 39.5-armLength};
    LOCATION_C = {78.5, 39.5-armLength};
    LOCATION_E = {9, 21-armLength };
    LOCATION_F = {78.5, 21-armLength};
}
}
}
