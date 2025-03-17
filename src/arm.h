#ifndef __ARM_AGENT__H
#define __ARM_AGENT__H 

#include "enviro.h"
#include <cmath>

using namespace enviro;

class armController : public Process, public AgentInterface {

    public:
        armController() : Process(), AgentInterface() {}

        void init() {
            Agent& node0 = add_agent("node", 0, 0, 0, {{"fill","green"},{"stroke","black"}});
            Agent& arm1 = add_agent("subarm", 0, 0, 0, {{"fill","blue"},{"stroke","black"}});
            Agent& node1 = add_agent("node2", 56, 0, 0, {{"fill","green"},{"stroke","black"}});
            Agent& arm2 = add_agent("subarm", 56, 0, 0, {{"fill","blue"},{"stroke","black"}});
            arm1.attach_to(node0);
            node1.attach_to(arm1);
            arm2.attach_to(node1);
        }
        void start() {}
        void update() {
            emit(Event("arm1_pos", json{{"x",x() + 50 * cos(angle())},{"y",y() + 50 * sin(angle())}}));
        }
        void stop() {}

};

class arm : public Agent {
    public:
    arm(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    armController c;
};

DECLARE_INTERFACE(arm)

#endif