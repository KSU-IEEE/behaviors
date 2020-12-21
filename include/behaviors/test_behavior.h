#ifndef TEST_BEHAVIOR_H
#define TEST_BEHAVIOR_H 

#include "behaviors/base_behavior.h"

namespace behaviors {
class test_behavior : public base_behavior {
public:
    test_behavior();
    ~test_behavior();

    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
private: 
};
} // namespace behaviors

#endif // TEST_BEHAVIOR_H