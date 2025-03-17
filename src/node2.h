#ifndef __NODE2_AGENT__H
#define __NODE2_AGENT__H 

#include "enviro.h"

using namespace enviro;

class node2Controller : public Process, public AgentInterface {

    public:
        double target_x = 0;
        double target_y = 0;
        node2Controller() : Process(), AgentInterface() {}
        
        void init() {
            watch("arm1_pos",[&](Event &e){
                target_x = e.value()["x"];
                target_y = e.value()["y"];
            });
        }
        void start() {}
        void update() {
            teleport(target_x,target_y, angle());
        }
        void stop() {}

};

class node2 : public Agent {
    public:
    node2(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    node2Controller c;
};

DECLARE_INTERFACE(node2)

#endif