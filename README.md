# behaviors
Location of behaviors for the IEEE autonomy stack.  
This is a standalone rospackage, that should never have any dependencies other than ros or basic c++ libraries.  

## compiling  
Because this is a standalone library, this can be compiled by doing the following:  
```BASH  
cd /path/to/behaviors  
mkdir build && cd build  
# ninja should always be the prefered build command  
cmake .. -G Ninja  
ninja  
```    

## Writing Behaviors  
This library is designed to be make writing behaviors easier. There is a base behavior that all subsequent behaviors sould inherit from. See test_behavior1.h for an example on how to write a behavior. The real things to worry about, are to have the following in your nodelet class:  
```c++  
class my_nodelet : public base_behavior {
public: 
    bool control_loop() override;
    void set_params() override;
    void nodelet_init() override;
}
```

## Running behaviors  
To run behaviors, you will need to still create plugins and launch file, follow the instruction in [this wiki](https://github.com/KSU-IEEE/wiki/blob/9-trevor-init-docs/lib/ros/writing_nodelet.md) for more information.  
After you do this, you will need to rebuild the code again:  
```BASH
cd /path/to/behaviors/build  
ninja # again, use make if that is what you had to use before
```

After rebuilding, you will need to source the new setup.sh file to export your package:  
```BASH  
source /path/to/behaviors/build/devel/setup.sh  
```  

Now you can run anything you've created in this repository with the roslaunch command. To run the test behviors, use this command: 
```BASH  
roslaunch behaviors test_behaviors.launch  
```  

