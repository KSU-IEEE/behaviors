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
void ground_search::finishedMove_cb(const std_msgs::Bool::ConstPtr& yes) {
    done_moving_ =  yes->data;
}

void ground_search::pos_cb(const behaviors::coordinate::ConstPtr& data) {
    // want the center of the body
    curr_x_ = data->X + 3;
    curr_y_ = data->Y + 3;
}

void ground_search::armDist_cb(const std_msgs::Float64::ConstPtr& val) {
    if(in_control()){
        bool ghost = calcExistance(val->data);
        if (ghost) {
            state = fsm::grabBlock;
        }
        arm_moving_ = false;
    }
}

void ground_search::armDone_cb(const std_msgs::Bool::ConstPtr& yes) {
    if (in_control() && state==fsm::grabBlock) {
        state = fsm::search;
    }
}


// internal funcs
/*************************************************************/
// utils

bool ground_search::calcExistance(float dist) {
    // compare it to what was sent 
    // see the one note for this
    float c_squared = r_sent*r_sent + baseHeight*baseHeight;
    float calcedDist = sqrt(c_squared - (armLength * armLength));
    float diff = abs(dist - calcedDist);
    if (diff < ghost_dist_tol_) return false;
    else return true;
}

bool ground_search::doneMoving() {
    // if finished moving after sending loc, done moving
    if (done_moving_ && sent_loc_) {
        sent_loc_ = false;
        done_moving_ = false;
        return true;
    }
    return false;
}

 behaviors::polar_coordinate ground_search::calc_message() {
     // based off of currentCheck
         // calc angle between locations
    // have to send in reguard to the distance from the outside of the bot
    // so distance from starting point
    float target_x = currentCheck.first;
    float target_y = currentCheck.second;

    float START_X = curr_x_;
    float START_Y = curr_y_;

    float x = abs(target_x - START_X);
    float y = abs(target_y - START_Y);
    float ang = atan(y/x);
    // qudrants 1 and 4

    if (target_x > START_X ) {
        if (target_y > START_Y) {
            // do nothing
        } else {
            // supplementary angle
            ang = 90 + ang;
        }
    } else {
        if (target_y > START_Y) {
            ang = 180 + (90 - ang);
        } else {
            ang = 270 + ang;
        }
    }

    // get distance
    float a = abs(target_x - x);
    float b = abs(target_y - y);
    float distance = sqrt(a*a + b*b); // hit pathagorean theorem

    // need to subtract distance from center
    distance = distance - armBaseLength;

    r_sent = distance;

    // send out coordinate
    behaviors::polar_coordinate msg;
    msg.theta = ang;
    msg.r = distance;

    return msg;
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
        msg.X = coord.first;
        msg.Y = coord.second;
        pub_goTo.publish(msg);

        // set sent location to true
        sent_loc_ = true;
    }

    return doneMoving();
}

bool ground_search::move() {
    // breakout if already sent loc
    if (sent_loc_) return false;

    // grab target
    behaviors::coordinate loc;
    if (doFirstBlock) {
        if (get_goal() == 'F') {
            loc.X = LOCATION_FS.first;
            loc.Y = LOCATION_FS.second;
        } else {
            loc.X = LOCATION_HS.first;
            loc.Y = LOCATION_HS.second;
        }
    } else {
        if (get_goal() == 'F') {
            loc.X = LOCATION_F.first;
            loc.Y = LOCATION_F.second;
        } else {
            loc.X = LOCATION_H.first;
            loc.Y = LOCATION_H.second;
        }
    }

    // send location 
    pub_goTo.publish(loc);

    sent_loc_ = true;
    done_moving_ = false;
    search_done_ = false;
}

// the meat of this guys
bool ground_search::search() {
    // treat each chunk differently
    // for first chunk, it's a 12 inch x 10 inch chunk
    // that starts one arm length away from the start point
    if(!arm_moving_){
    float x = curr_x_;
    float y = curr_y_;

    float boundx, boundy;
    if(doFirstBlock) {
        // first set initial
        if (!setInitialPoint) {
            currentCheck.first = x - 5.5;
            currentCheck.second = y - armLength;
            initialCheck = currentCheck;
            setInitialPoint = false;
        }
    } else {
        // first set initial
        if (!setInitialPoint) {
            currentCheck.first = x - armLength;
            currentCheck.second = y - 5;
            initialCheck = currentCheck;
            setInitialPoint = false;
        }
    }
    boundy = initialCheck.second - 9.5; // round down
    if (left) {
        boundx = initialCheck.first;
        currentCheck.first -= 1;

        if (currentCheck.first < boundx) {
            left = false;
            currentCheck.first += 1;
            currentCheck.second -= 1;
        }
    } else {
        boundx = initialCheck.first + 11.5;
        currentCheck.first += 1;

        if (currentCheck.first > boundx) {
            left = true;
            currentCheck.first -= 1;
            currentCheck.second -= 1;
        }
    }

    // check y bounds
    if(currentCheck.second < boundy) { // this block is done
        // continue to next block if first done
        if (doFirstBlock) {
            doFirstBlock = false;
            move_ = true;
        } else {
            doFirstBlock = true;
            search_done_ = true;
        }
        return true;
    }

    // if still in bounds, send next check
    behaviors::polar_coordinate msg = calc_message();
    pub_armScan.publish(msg);

    // wait to do anything else until you check distance 
    arm_moving_ = true;
    }
}


bool ground_search::grabBlock() {
    // move to correct distance
    if (!sent_loc_ && !done_moving_) {
        // different for the block you are checking

        behaviors::coordinate msg;
        if (doFirstBlock) {
            msg.X = (get_goal() == 'F') ? 6.5 : 87.5;
            msg.Y = currentCheck.second + armLength;
        } else {
            msg.X = (get_goal() == 'F') ? currentCheck.first - armLength : currentCheck.first + armLength;
            msg.Y = 5.5;
        } 

        pub_goTo.publish(msg);
        sent_loc_ = true;
    } else if (!arm_moving_) {
        // send grab messaage
        // calc angle
        behaviors::polar_coordinate temp = calc_message();
        std_msgs::Float64 msg;
        msg.data = temp.theta;
        pub_armGrab.publish(msg);
        arm_moving_ = true;
    }
}

bool ground_search::checkTransition() {
    switch(state){
        case (fsm::moving):
            // determine to transition
            if (sent_loc_ && done_moving_) {
                state = fsm::search;
                sent_loc_ = false;
                done_moving_ = false;
            }
            break;
        case (fsm::search):
            // need to handle moving to the second location
            if (move_ || search_done_){
                state = fsm::moving;
            }
            break;
        case (fsm::grabBlock):
            // handles transition automatically
            break;
    }
}

// override funcs
/*************************************************************/
bool ground_search::control_loop() {
    switch(state) {
        case (fsm::moving):
            move();
            break;
        case (fsm::search):
            search();
            break;
        case (fsm::grabBlock):
            grabBlock();
            break;
    }

    checkTransition();  // udpate fsm

    return search_done_;  // set in search
}
void ground_search::set_params() {
    nh().getParam("botWidth", botWidth);
    nh().getParam("/arm/length", armLength);
    nh().getParam("/arm/baseHeight", baseHeight);
    nh().getParam("/arm/baseLength", armBaseLength);
    

}
void ground_search::nodelet_init() {
    // init subs 
    sub_finishedMove = nh().subscribe("moveDone" , 1000, &ground_search::finishedMove_cb, this);
    sub_posiiton     = nh().subscribe("pose"     , 1000, &ground_search::pos_cb         , this);
    sub_armDist = nh().subscribe("/arm/distance", 1000, &ground_search::armDist_cb, this);
    sub_armDon = nh().subscribe("/arm/done", 1000, &ground_search::armDone_cb, this);

    // init pubs
    pub_goTo = nh().advertise<behaviors::coordinate>("moveTo", 1000);
    pub_armScan = nh().advertise<behaviors::polar_coordinate>("/arm/scan", 1000);
    pub_armGrab = nh().advertise<std_msgs::Float64>("/arm/grabBlock", 1000);

    LOCATION_FS = { 3 , 10.5 + armLength};
    LOCATION_F = {12.5 - armLength, 2};
    LOCATION_HS = { 84.5 , 10.5 + armLength};
    LOCATION_H = {81.5 + armLength, 2};
}
} // pac_man_behs
} // behaviors