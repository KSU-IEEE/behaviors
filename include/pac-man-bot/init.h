#ifndef INIT_H
#define INIT_H

#include <base_behavior.h>

namespace behaviors {
namespace pac_man_behs {
class init : base_behavior {
public:
    init();
    ~init();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private:


};  // init
} // pac_man_behs
} // behaviors


#endif  // INIT_H