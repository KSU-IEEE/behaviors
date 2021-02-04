#ifndef ATTACK_GHOSTS_H
#define ATTACK_GHOSTS_H

#include <base_behavior.h>

namespace behaviors {
namespace pac_man_behs {
class attack_ghosts : base_behavior {
public:
    attack_ghosts();
    ~attack_ghosts();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private:


};  // attack_ghosts
} // pac_man_behs
} // behaviors


#endif  // ATTACK_GHOSTS_H