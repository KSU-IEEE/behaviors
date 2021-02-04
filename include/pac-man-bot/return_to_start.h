#ifndef RETURN_TO_START_H
#define RETURN_TO_START_H

#include <base_behavior.h>

namespace behaviors {
namespace pac_man_behs {
class return_to_start : base_behavior {
public:
    return_to_start();
    ~return_to_start();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private:


};  // return_to_start
} // pac_man_behs
} // behaviors


#endif  // RETURN_TO_START_H