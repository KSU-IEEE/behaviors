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
    0       G4      64.0     153.5   
    1       G3      28.0     153.5
    2       G1      28.0     133.5
    3       G5      38.0     133.5
    4       G2      64.0     133.5  // won't use this one becasue
                                       it's the last case
    */
    // yes I'm hard coding this. based on .5 inch positions
    // starting at the top left of the map
    locs_.push_back(std::make_pair(64.0, 153.5));
    locs_.push_back(std::make_pair(28.0, 153.5));
    locs_.push_back(std::make_pair(28.0, 133.5));
    locs_.push_back(std::make_pair(38.0, 133.5));
    locs_.push_back(std::make_pair(64.0, 133.5));
}
check_for_ghosts::~check_for_ghosts() {

}

// callbacks
void check_for_ghosts::distances_cb(const behaviors::distances::ConstPtr& msg) {
    // to save some threads, don't do this everytime
    if(in_control()){
        front_dist_ = msg -> front.data;
        back_dist_ =  msg -> backward.data;
        left_dist_ =  msg -> left.data;
        right_dist_ = msg -> right.data;
    }
}

// class specific functions
bool check_for_ghosts::checkGhosts() {
    // a couple of edge cases here. First,
    // the oritation to check G1-G4 are different
    // than that of G5 (see comment in initializer
    //                  for indexes)
    bool found = false;
    if (curr_target != 3) {
        // different for each cardinal direction
        if (heading_ == 0 || heading_ == 360) {
            found = (front_dist_ < ghost_dist_tol_) ? true : false;
        } else if (heading_ == 90) {
            found = (left_dist_ < ghost_dist_tol_) ? true : false;            
        } else if (heading_ == 180) {
            found = (back_dist_ < ghost_dist_tol_) ? true : false;
        } else { 
            found = (right_dist_ < ghost_dist_tol_) ? true : false;
        }
    } else {
        // again check the four directions
        if (heading_ == 0 || heading_ == 360) {
            found = (right_dist_ < ghost_dist_tol_) ? true : false;
        } else if (heading_ == 90) {
            found = (front_dist_ < ghost_dist_tol_) ? true : false;
        } else if (heading_ == 180) {
            found = (back_dist_ < ghost_dist_tol_) ? true : false;
        } else { 
            found = (left_dist_ < ghost_dist_tol_) ? true : false;
        }
    }
    return found;
}

void check_for_ghosts::sendMoveCmd() {
    // first incremet index, and unset done_moving_
    curr_target++;
    done_moving_ = false;

    behaviors::coordinate msg;
    msg.X.data = locs_.at(curr_target).first;
    msg.Y.data = locs_.at(curr_target).second;
    
    pubGoTo.publish(msg);
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

    pubGhostLoc.publish(msg);
}


// base funcs 
bool check_for_ghosts::control_loop(){
    bool ghost1 = ghost1_;
    bool ghost2 = ghost2_;


    if(done_moving_){
        // check them ghosts
        if (!ghost1 && curr_target >= 0){
            ghost1 = checkGhosts();
            sendGhostLoc();
        }
        else if (!ghost2 && curr_target >= 0) {
            ghost2 = checkGhosts();
        }
        // no need to check the last location, we know it's the last one
        if (!ghost2 && curr_target == locs_.size() - 1) {
            ghost2 = true;
            curr_target++;
        }

        // send location if found
        if (ghost1_ != ghost1 || ghost2_ != ghost2) {
            print_msg("Found Ghost");
            sendGhostLoc();
        } 

        // move if not both found
        if (!ghost1 && !ghost2)
            sendMoveCmd();
    }
    
    ghost1_ = ghost1;
    ghost2_ = ghost2;

    return ghost1_ && ghost2_;
}

void check_for_ghosts::set_params(){
    // need to get tol for ghost distances
}

void check_for_ghosts::nodelet_init(){
    // init subs
    subDistances    = nh().subscribe("distances", 100, &check_for_ghosts::distances_cb   , this);
    subHeading      = nh().subscribe("heading"  , 100, &check_for_ghosts::heading_cb     , this);
    subFinishedMove = nh().subscribe("moveDone" , 100, &check_for_ghosts::finishedMove_cb, this);

    //init pubs
    pubGhostLoc = nh().advertise<std_msgs::Int8>       ("ghostLocation", 1000);
    pubGoTo     = nh().advertise<behaviors::coordinate>("moveTo"       , 1000);
}


} // pac_man_behs
} // behaviors