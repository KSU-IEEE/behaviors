#include <pac-man-bot/end.h>

PLUGINLIB_EXPORT_CLASS(behaviors::pac_man_behs::end_beh, nodelet::Nodelet);

namespace behaviors {
namespace pac_man_behs {
end_beh::end_beh() : base_behavior{"end"} {

}

end_beh::~end_beh() {

}

// override
bool end_beh::control_loop() {
    return false;
}

void end_beh::set_params() {

}

void end_beh::nodelet_init() {

}


}
} // behaviors