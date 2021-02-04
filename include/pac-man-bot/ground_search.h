#ifndef GROUND_SEARCH_H
#define GROUND_SEARCH_H

#include <base_behavior.h>

namespace behaviors {
namespace pac_man_behs {
class ground_search : base_behavior {
public:
    ground_search();
    ~ground_search();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private:


};  // ground_search
} // pac_man_behs
} // behaviors


#endif  // GROUND_SEARCH_H