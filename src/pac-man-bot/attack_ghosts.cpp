#include <pac-man-bot/attack_ghosts.h>

// export
PLUGINLIB_EXPORT_CLASS(behaviors::pac_man_behs::attack_ghosts, nodelet::Nodelet);

namespace behaviors {
namespace pac_man_behs {
// inits and deinits
/*******************************************************************************/
attack_ghosts::attack_ghosts() : base_behavior{"attack_ghosts"}{

}

attack_ghosts::~attack_ghosts() {

}

// ros callbacks
/*******************************************************************************/
void attack_ghosts::ghostLoc_cb(const std_msgs::Int8::ConstPtr& loc) {
    switch (loc->data) {
        case 1:
            ghost1_ = true;
            break;
        case 2: 
            ghost2_ = true;
            break;
        case 3: 
            ghost3_ = true;
            break;
        case 4:
            ghost4_ = true;
            break;
        case 5:
            ghost5_ = true;
            break;
        default:
            break;
    }
}

void attack_ghosts::armDone_cb(const std_msgs::Bool::ConstPtr& yes) {
    grabbing = !yes->data;

    // update ghosts list 
    // go through ghosts and unset the top most
    // send grabbed success
    std_msgs::Int8 taken;
    if (yes->data) {
        if (ghost1_){
            ghost1_ = false;
            taken.data = 1;
        } else if (ghost2_) {
            ghost2_ = false;
            taken.data = 2;
        } else if (ghost3_) {
            ghost3_ = false;
            taken.data = 3;
        } else if (ghost4_) {
            ghost4_ = false;
            taken.data = 4;
        } else if (ghost5_){
            ghost5_ = false;
            taken.data = 5;
        }
        print_msg("Grabbed dat ghost");
        pub_grabbedGhost.publish(taken);
    }

    grabbing = false;
}


void attack_ghosts::heading_cb(const std_msgs::Float64::ConstPtr& degree) {
    heading_ = degree->data;
}

void attack_ghosts::finishedMove_cb(const std_msgs::Bool::ConstPtr& yes) {
    finished_move_ = yes->data;
}

void attack_ghosts::pose_cb(const behaviors::coordinate& loc) {
    x = loc.X;
    y = loc.Y;
}


// internal functions
/*******************************************************************************/
bool attack_ghosts::stopwatch(int seconds) {
    std::chrono::duration<float> diff = 
        std::chrono::system_clock::now() - start_time_;

    // send status
    int secs = int(floor(diff.count()));
    if (secs % 5 == 0 && secs != last_msg_) {
        print_msg("has waited for " + std::to_string(secs) + " seconds");
        last_msg_ = secs;
    }
    return diff.count() >= seconds;
}

bool attack_ghosts::move()  {
    // create condiitoins
    if (!sent_loc_ && !grabbing) {
        // send locations, prioritze top, then bottome, then middle
        // becuase of where you will be starting (at a or c) and that
        // middle will put you right above the starting box

        behaviors::coordinate msg;

        if (ghost1_) {
            msg.X = GHOST_LOC1.first - 3;
            msg.Y = GHOST_LOC1.second + distancePickUp_;
        } else if (ghost2_) {
            msg.X = GHOST_LOC2.first - 3;
            msg.Y = GHOST_LOC2.second + distancePickUp_;
        } else if (ghost3_) {
            msg.X = GHOST_LOC3.first - 3;
            msg.Y = GHOST_LOC3.second + distancePickUp_;
        } else if (ghost4_) {
            msg.X = GHOST_LOC4.first - 3;
            msg.Y = GHOST_LOC4.second + distancePickUp_;
        } else {
            msg.X = GHOST_LOC5.first - distancePickUp_;
            msg.Y = GHOST_LOC5.second - 3;
        }

        pub_goTo.publish(msg);
        sent_loc_ = true;
        return false;
    } else if(!finished_move_) {
        return false;
    } else {
        start_grabbing = true;
        sent_loc_ = false;
        return true;
    }
}

bool attack_ghosts::grab()  {
    if (start_grabbing) {
        // calc angle 
        float target_x, target_y;
        if (ghost1_) {
            target_x = GHOST_LOC1.first;
            target_y = GHOST_LOC1.second;
        } else if (ghost2_) {
            target_x = GHOST_LOC2.first;
            target_y = GHOST_LOC2.second;
        } else if (ghost3_) {
            target_x = GHOST_LOC3.first;
            target_y = GHOST_LOC3.second;
        } else if (ghost4_) {
            target_x = GHOST_LOC4.first;
            target_y = GHOST_LOC4.second;
        } else {
            target_x = GHOST_LOC5.first;
            target_y = GHOST_LOC5.second;
        }

        float ang = atan(abs(target_y - y)/abs(target_x - x));
        // qudrants 1 and 4
        if (target_x > x ) {
            if (target_y > y) {
                // do nothing
            } else {
                // supplementary angle
                ang = 90 + ang;
            }
        } else {
            if (target_y > y) {
                ang = 180 + (90 - ang);
            } else {
                ang = 270 + ang;
            }
        }

        // send off angle
        std_msgs::Float64 msg;
        msg.data = ang;
        pub_armGrabBlock.publish(msg);

        // set control vars
        start_grabbing = false;
        grabbing = true;
    }
    
    return false; //temp
}

// overrides
/*******************************************************************************/    
bool attack_ghosts::control_loop() {
    // start timer
    if (initialRun_) {
        print_msg("Starting Timer");
        start_time_ = std::chrono::system_clock::now();
        initialRun_ = false;
    }

    // check if out of time
    timer_ = stopwatch(30);

    // actual state here
    // start by moving to desired location 

    // changing this, we only need to move and grab
    move();
    grab();

    ghosts_killed_ = !ghost1_ && !ghost2_ && !ghost3_ && !ghost4_ && !ghost5_;

    bool testing = timer_ || ghosts_killed_;

    //cout<<"I'm inside attack_ghost, returning " << testing <<endl;
    return timer_ || ghosts_killed_;
}

void attack_ghosts::set_params() {
    float armlength, armBase;
    nh().getParam("/arm/length", armlength);
    nh().getParam("/arm/baseHeight", armBase);

    distancePickUp_ = acos(armBase / armlength); // see oneNote
}

void attack_ghosts::nodelet_init() {
    // init subs
    sub_ghostLoc = nh().subscribe("ghostLocation", 100, &attack_ghosts::ghostLoc_cb, this);
    sub_finishedMove = nh().subscribe("moveDone", 100, &attack_ghosts::finishedMove_cb, this); 
    sub_heading = nh().subscribe("heading", 100, &attack_ghosts::heading_cb, this);
    sub_armDone = nh().subscribe("/arm/done", 100, &attack_ghosts::armDone_cb, this);
    sub_pose = nh().subscribe("position", 100, &attack_ghosts::pose_cb, this);

    // init pubs
    pub_goTo = nh().advertise<behaviors::coordinate>("moveTo", 1000);
    pub_setHeading = nh().advertise<std_msgs::Float64>("setHeading", 1000);
    pub_armGrabBlock = nh().advertise<std_msgs::Float64>("/arm/grabBlock", 1000);
    pub_grabbedGhost = nh().advertise<std_msgs::Int8> ("grabbedGhost", 100);
}
}// pac_man_behs
}//  behaviors