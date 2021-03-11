#include <pac-man-bot/check_for_ghosts.h>

// plugin
PLUGINLIB_EXPORT_CLASS(behaviors::pac_man_behs::check_for_ghosts, nodelet::Nodelet);

namespace behaviors {
namespace pac_man_behs {
check_for_ghosts::check_for_ghosts() : 
    base_behavior {"check_for_ghosts"} {
    /*
    populate ghost vector of pairs
    
    list of locations to check for ghosts:
    id   id on map    x      y
    0       G4      64.6     11.5  
    1       G3      29.5     11.5
    2       G1      29.5     25.5
    3       G5      46.5     16.0
    4       G2      64.5     25.5  // won't use this one becasue
                                       it's the last case
    */
    // yes I'm hard coding this. based on .5 inch positions
    // starting at the top left of the map
    locs_.push_back(std::make_pair(46.4, 16.0));
    locs_.push_back(std::make_pair(64.5, 25.5));
    locs_.push_back(std::make_pair(64.5, 11.5));
    locs_.push_back(std::make_pair(29.5, 11.5));
    locs_.push_back(std::make_pair(29.5, 25.5));
}
check_for_ghosts::~check_for_ghosts() {

}

// callbacks
void check_for_ghosts::armDistance_cb(const std_msgs::Float64::ConstPtr& dist) {
    if(in_control()) {
    bool ghost = calcExistance(dist->data);
    if (ghost) { 
        sendGhostLoc();
        if (!ghost1_) ghost1_ = true;
        else ghost2_ = true;
    }
    curr_target++;
    sent_loc = false;
    }
}

void check_for_ghosts::sendGhostLoc() {
    std_msgs::Int8 msg;
    //decode from index, see comment in init
    switch(curr_target){
        case 0:
            msg.data = 4;
            break;
        case 1:
            msg.data = 3;
            break;
        case 2:
            msg.data = 1;
            break;
        case 3: 
            msg.data = 5;
            break;
        default:
            msg.data = 2;
            break;
    }

    pub_ghostLoc.publish(msg);
}

void check_for_ghosts::moveArm() {
    // grab target ghost
    float target_x = locs_.at(curr_target).first;
    float target_y = locs_.at(curr_target).second;

    // calc angle between locations
    // have to send in reguard to the distance from the outside of the bot
    // so distance from starting point
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
    pub_scan.publish(msg);

    // set control var
    sent_loc = true;
}

bool check_for_ghosts::calcExistance(float dist) {
    // compare it to what was sent 
    float diff = dist - r_sent;
    if (diff < .5 && diff > -.5) return false;
    else return true;
}


// base funcs 
bool check_for_ghosts::control_loop(){

    // rethinking this
    if (!sent_loc) moveArm();

    return ghost1_ && ghost2_;
}

void check_for_ghosts::set_params(){
    nh().getParam("starting_x", START_X);
    nh().getParam("starting_y", START_Y);
    nh().getParam("arm/baseLength", armBaseLength);
}

void check_for_ghosts::nodelet_init(){
    // init subs
    sub_armDistance = nh().subscribe("/arm/distance", 100, &check_for_ghosts::armDistance_cb, this);

    //init pubs
    pub_scan = nh().advertise<behaviors::polar_coordinate>("/arm/scan", 1000);
    pub_ghostLoc = nh().advertise<std_msgs::Int8>("ghostLocation", 1000);
}


} // pac_man_behs
} // behaviors