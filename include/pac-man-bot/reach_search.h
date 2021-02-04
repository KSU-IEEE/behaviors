#ifndef REACH_SEARCH_H
#define REACH_SEARCH_H

#include <base_behavior.h>

namespace behaviors {
namespace pac_man_behs {
class reach_search : base_behavior {
public:
    reach_search();
    ~reach_search();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private:


};  // reach_search
} // pac_man_behs
} // behaviors


#endif  // REACH_SEARCH_H