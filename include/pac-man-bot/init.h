#ifndef INIT_H
#define INIT_H
#include <base_behavior.h>

/**************************************************
pac-man-bot::init:
    The first state in the pac-man-bot statemachine.
    This behavior will control two 
    things. First, it will control the switch waiting
    period. This state will initialize on startup, and
    wait for the start switch to be pressed. Secondly,
    it will wait 30 seconds during the communication
    phase. After both of these are completed, it will
    release control.

End statement: 
    begin_ && (power_loc_ || time_out_)
    Switch flipped and the power location is sent or
    the 30 second period is over
===================================================
CONNECTIONS
subscribers: 
    start_signal
    PP_location

publishers:
    start_listen
**************************************************/

// ros includes
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>

// random functions
#include <chrono> // used for a timer (count to 30 s)
#include <math.h>

namespace behaviors {
namespace pac_man_behs {
class init : public base_behavior {
public:
    init();
    ~init();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;

    // subscribers && callbacks:
    ros::Subscriber subStartSignal, subPPLoc;
    void startSignal_cb(const std_msgs::Bool::ConstPtr& start);
    void ppLoc_cb (const std_msgs::Char::ConstPtr& loc);

    // publishers
    ros::Publisher pubStartListen;

    // internal funcs 
    bool stopwatch(int seconds);

private:
    // end conditions
    bool begin_ = false;
    bool power_loc_;
    bool time_out_;  

    // timer componenets
    const int THIRTY_SECS = 29; // round down to be sure
    std::chrono::time_point<std::chrono::system_clock> start_time_;
    int last_msg_ = -1;

};  // init
} // pac_man_behs
} // behaviors


#endif  // INIT_H