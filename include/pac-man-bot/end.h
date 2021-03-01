#ifndef END_H
#define END_H

#include <base_behavior.h>
/**************************************************
pac-man-bot::end

This class just sits there

End statement: none
==================================================
CONNECTIONS
subscribers: none

publishers: none
**************************************************/

namespace behaviors {
namespace pac_man_behs {
class end_beh : public base_behavior {
public:
    end_beh();
    ~end_beh();

    // override
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private:


};  // end_beh
} // pac_man_behs
} // behaviors


#endif  // END_H