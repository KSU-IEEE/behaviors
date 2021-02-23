#include <pac-man-bot/ground_search.h>

// make visible
PLUGINLIB_EXPORT_CLASS(behaviors::pac_man_behs::ground_search, nodelet::Nodelet);


namespace behaviors {
namespace pac_man_behs {
ground_search::ground_search()
    : base_behavior {"ground_search"} {
}

ground_search::ground_search(std::string overrideName) 
    : base_behavior {overrideName} {

}

ground_search::~ground_search() {

}

// callbacks 
/*************************************************************/
void ground_search::heading_cb(const std_msgs::Float64::ConstPtr& degree) {
    heading_ = degree->data;
}

void ground_search::finishedMove_cb(const std_msgs::Bool::ConstPtr& yes) {
    done_moving_ =  yes->data;
}

void ground_search::distances_cb(const behaviors::distances::ConstPtr& data) {
    front_dist_ = data->front.data;
    back_dist_  = data->backward.data;
    left_dist_  = data->left.data;
    right_dist_ = data->right.data;
}

void ground_search::pos_cb(const behaviors::coordinate::ConstPtr& data) {
    curr_x_ = data->X.data;
    curr_y_ = data->Y.data;
}


// internal funcs
/*************************************************************/
// utils
bool ground_search::doneMoving() {
    // if finished moving after sending loc, done moving
    if (done_moving_ && sent_loc_) {
        sent_loc_ = false;
        done_moving_ = false;
        return true;
    }
    return false;
}
// search functions
bool ground_search::checkHeading(int head) {
    // set heading first -- only send once
    if (heading_ != head && ! waiting_for_head_) {
        std_msgs::Float64 msg;
        msg.data = 0;
        pub_setHeading.publish(msg);

        waiting_for_head_ = true;
        return false;
    }
    
    // check heading if needed
    waiting_for_head_ = (waiting_for_head_ && (heading_ == 0)) ? 
                        false : true;
    return waiting_for_head_;
}

bool ground_search::next() {
    if (!sent_loc_) {
        // make target and send
        behaviors::coordinate msg;
        msg.X.data = curr_x_;
        msg.Y.data = curr_y_ - .5;
        pub_goTo.publish(msg);

        sent_loc_ = true;
    }

    return doneMoving();
}

bool ground_search::check() {
    // clear ready to check for next iteration
    ready_to_check_ = false;

    // check in front of the first iteration
    if (!checked_ahead_) {
        if (back_dist_ <= 12.7)
            return true;
        else {
            checked_ahead_ = true;
            return false;
        }
    }

    // check different sides depending on target
    // max distance of 10"
    switch(get_goal()) {
        case 'F':
            // check right side. give some tolerance
            if (right_dist_ <= 9.7){
                
                return true;
            }
            break;
        case 'H':
            if (left_dist_ <= 9.7){
                return true;
            }
            break;
    }
    return false;
}

// fsm high level functions
bool ground_search::moveToStart() {
    // set return param during this, so it won't quit after
    // the first run
    search_done_ = false;

    if (!sent_loc_) {
        // two different locations to go to based on target givin
        std::pair<float, float> coord;  // x, y
        switch(get_goal()) {
            case 'F':
                coord = LOCATION_F;
                break;
            case 'H':
                coord = LOCATION_H;
                break;
            default: 
                break;
        }

        // pack and send message
        behaviors::coordinate msg;
        msg.X.data = coord.first;
        msg.Y.data = coord.second;
        pub_goTo.publish(msg);

        // set sent location to true
        sent_loc_ = true;
    }

    return doneMoving();
}

bool ground_search::search() {
    // check if heading is 0
    if (!checkHeading(TARGET_HEADING)) return false;

    // heading is correct, now start search
    // do a series of small searches every .5 inches
    // every row should be the next .5 inch below, no matter
    // current target
    bool returner;
    if (!ready_to_check_) {
        ready_to_check_ = next();  // send message to move or wait until moved
        return false;
    } else {
        returner = check();
    }

    // check if done with pattern
    if (curr_y_ > 42.5 && !returner) search_done_ = true;

    return returner;
}
bool ground_search::grabBlock() {
    // two conditions. initial block check when check_ahead_ == false
    // and normal to the side
}

// override funcs
/*************************************************************/
bool ground_search::control_loop() {
    bool transition;
    // internal fsm 
    // go to state, and handle transitions
    switch(state){
        case fsm::moveToStart:
            transition = moveToStart();
            if (transition){
                state = fsm::search;
                print_msg("Reached top of box, switching to search");
            } 
            break;
        case fsm::search:
            transition = search();
            if (transition && !search_done_) {
                state = fsm::grabBlock;
                print_msg("Found block, switching to grabBlock");
            } else if (transition && search_done_) {
                // return fsm to moveToStart for next iteration
                state = fsm::moveToStart;
                print_msg("Search complete, giving up control");
            }
            break;
        case fsm::grabBlock:
            transition = grabBlock();
            if (transition) {
                state = fsm::search;
                print_msg("Returned to track, continueing search");
            }
        default:
            break;
    }

    return search_done_;
}
void ground_search::set_params() {
    // tbd
}
void ground_search::nodelet_init() {
    // init subs 
    sub_finishedMove = nh().subscribe("moveDone" , 1000, &ground_search::finishedMove_cb, this);
    sub_heading      = nh().subscribe("heading"  , 1000, &ground_search::heading_cb     , this);
    sub_distances    = nh().subscribe("distances", 1000, &ground_search::distances_cb   , this);
    sub_posiiton     = nh().subscribe("pose"     , 1000, &ground_search::pos_cb         , this);

    // init pubs
    pub_goTo = nh().advertise<behaviors::coordinate>("moveTo", 1000);
    pub_setHeading = nh().advertise<std_msgs::Float64>("setHeading", 1000);
}
} // pac_man_behs
} // behaviors