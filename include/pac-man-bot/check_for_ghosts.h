#ifndef CHECK_FOR_GHOSTS_H
#define CHECK_FOR_GHOSTS_H

#include <base_behavior.h>

namespace behaviors {
namespace pac_man_behs {
class check_for_ghosts : base_behavior {
public:
    check_for_ghosts();
    ~check_for_ghosts();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private:


};  // check_for_ghosts
} // pac_man_behs
} // behaviors


#endif  // CHECK_FOR_GHOSTS_H