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
void reach_search::armDistance_cb(const std_msgs::Float64::ConstPtr& dist){
    arm_dist_ = dist->data;
}

// override - from ground_search
/**************************************************************/
// utils
bool reach_search::doneWithSearch() {
    // doneWithSearch is different for each different search box
    switch (get_goal()) {
    case 'A':
        /* 
            case:
            at end of A and Arm is over last square of wall
        */
        return (curr_x_ >= LOCATION_A.first + 14 && 
                arm_loc_.first == 18 &&
                arm_loc_.second == 8);
        break;
    case 'B':
        /*
            case:
            at end of B and Arm is over last square
        */
        return (curr_x_ >= LOCATION_B.first + 11 &&
                arm_loc_.first == LOCATION_B.first + 11 &&
                arm_loc_.second == 8);
        break; 
    case 'C':
        return (curr_x_ >= LOCATION_C.first + 13 &&
                arm_loc_.first == .5 &&
                arm_loc_.second == .5);
        break;
    case 'D':
        return (curr_x_ >= LOCATION_D.first + 19 &&
                arm_loc_.first == LOCATION_D.first + 19 &&
                arm_loc_.second == LOCATION_D.second - 4);
        break;
    case 'E':
        return (curr_x_ >= LOCATION_E.first + 16 &&
                arm_loc_.first == LOCATION_E.first + 19 &&
                arm_loc_.second == LOCATION_E.second);
        break;
    default:
        // return true: bad location, go to next state
        return true;
}
}

bool reach_search::initialArmPos() {
    // again different position based on goal
    return arm_done_moving_;
}

void reach_search::moveArm() {
    // assumes arm & bot are done moving (see next())
    // different lengths for ABC and DE
    if (get_goal() == 'A' || get_goal() == 'B' || get_goal() == 'C') {
        // is arm moving out or in?
        if (search_outward_) { // moving out
            // handle conversion with search_outward_ here,
            // you want it at the beginning to be at the outward position twice
            search_outward_ = curr_count_col_ >= LENGTH_OF_ABC ? false : true;

            // get current target column distance, edge cases with the wall in the way
            if (get_goal() == 'A' && num_with_wall_ < ROWS_TO_SEARCH_AT_WALL) {
                if (curr_count_col_ >= LENGTH_OF_ABC) {
                    num_with_wall_++;
                    move_ = num_with_wall_ < ROWS_TO_SEARCH_AT_WALL;
                }
            } else if (get_goal() == 'C' && num_with_wall_ < ROWS_TO_SEARCH_AT_WALL) {
                if (curr_count_col_ >= LENGTH_OF_ABC) {
                    num_with_wall_++;
                    move_ = false;
                }
            } else {
                move_ = (curr_count_col_ >= LENGTH_OF_ABC);
            }

            // incrememnt column if necesarry 
            curr_count_col_ = (curr_count_col_ <LENGTH_OF_ABC) ?
                                curr_count_col_++ : curr_count_col_;

        } else { // moving in 
            // hand conversion with search_outward_ here
            search_outward_ = curr_count_col_ <= .5 ? true : false;

            // get current target column distance, edge cases with the wall in the way
            if ((get_goal() == 'A' || get_goal() == 'C') && num_with_wall_ < ROWS_TO_SEARCH_AT_WALL) {
                // combine A and C results are the same, move should always be false
                move_ = false;
                num_with_wall_ = curr_count_col_ >= 0.5 ? num_with_wall_ : num_with_wall_++;
            } else {
                move_ = (curr_count_col_ <= .5);
            }

            // decrement count if needed
            curr_count_col_ = curr_count_col_ >= .5 ? 
                              curr_count_col_-- : curr_count_col_;
        }
    } else {
        if (search_outward_){
            // hand search outward 
            search_outward_ = curr_count_col_ >= LENGTH_OF_DE ? false : true;
            // different edge case for D at the beginning
            if (get_goal() == 'D' && num_with_wall_ < ROWS_TO_SEARCH_AT_WALL) {
                if (curr_count_col_ >= LENGTH_OF_DE) {
                    num_with_wall_++;
                    move_ = num_with_wall_ < ROWS_TO_SEARCH_AT_WALL;
                }
            } else {
                move_ = curr_count_col_ >= LENGTH_OF_DE;
            }

            // increment count
            curr_count_col_ = curr_count_col_ < LENGTH_OF_DE ?
                              curr_count_col_++ : curr_count_col_;
        } else {
            if (get_goal() == 'D' && num_with_wall_ < ROWS_TO_SEARCH_AT_WALL) {
                move_ = false;
                num_with_wall_ = curr_count_col_ >= 0.5 ? num_with_wall_ : num_with_wall_++;
            } else {
                move_ = curr_count_col_ <= .5;
            }

            // decrement if needed
            curr_count_col_ = curr_count_col_ >= .5 ? 
                              curr_count_col_-- : curr_count_col_;
        }
    }

    // TODO: pack up arm message here
}

void reach_search::moveBot() {
    if (move_) {
        // pack up coordinate msg 
        behaviors::coordinate msg;
        msg.X = curr_x_ + .5;
        msg.Y = curr_y_;
        pub_goTo.publish(msg);
    }
}

bool reach_search::next() {
    // edge cases: arm has to move and body has to move
    // treat it differently for boxes ABC and DE
    
    // first should you be moving the arm or body?
    // first is arm.....just so you know
    // diffent again for every freaking target
    if (initial_box_search_) {
        // set the arm, wait for it to be done moving
        initial_box_search_ = initialArmPos();
        curr_count_col_ = 0;
        move_ = false;
        return false;
    }

    // normal situtation
    // wait for arm to stop moving
    if (!arm_done_moving_ && !done_moving_) {
        return false;
    } else {
        // check which direction to put the arm
        moveArm();
        moveBot();
    }

}

bool reach_search::check() {
    search_done_ = doneWithSearch();

    return arm_dist_ < arm_to_ground_ ? true : false;
}

// fsm funcs
bool reach_search::moveToStart() {
    // set search_done_ to false for next iteration
    search_done_ = false;

    // updated initial_box_search to true for next fms state
    initial_box_search_ = true;

    // 5 possible locations to be sent to - ABCDE
    behaviors::coordinate msg;
    if (!sent_loc_) {
        switch(get_goal()){
        case 'A':
            msg.X = LOCATION_A.first;
            msg.Y = LOCATION_A.second;
            break;
        case 'B':
            msg.X = LOCATION_B.first;
            msg.Y = LOCATION_B.second;
            break;
        case 'C':
            msg.X = LOCATION_C.first;
            msg.Y = LOCATION_C.second;
            break;
        case 'D':
            msg.X = LOCATION_D.first;
            msg.Y = LOCATION_D.second;
            break;
        case 'E':
            msg.X = LOCATION_E.first;
            msg.Y = LOCATION_E.second;
            break;
        default: break;
        }
        // send location
        pub_goTo.publish(msg);

        // set for next iteration
        sent_loc_ = true;
    }

    return doneMoving();
}

bool reach_search::search() {
    if (! doneMoving() && !arm_done_moving_) {
        return false;
    }

    next();
    return check();
}

bool reach_search::grabBlock() {
 // todo
}

// override functions - from base_behavior
/**************************************************************/
void reach_search::set_params() {
    nh().getParam("/reach_search/targetArmDistance", arm_to_ground_);
}

void reach_search::nodelet_init() {
    ground_search::nodelet_init();
}
}
}
