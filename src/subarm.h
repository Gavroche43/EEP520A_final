#ifndef __SUBARM_AGENT__H
#define __SUBARM_AGENT__H 

#include "enviro.h"

using namespace enviro;

class subarmController : public Process, public AgentInterface {

    public:
    subarmController() : Process(), AgentInterface() {}

    void init() {}
    void start() {}
    void update() {
        track_velocity(0,1);
    }
    void stop() {}

};

class subarm : public Agent {
    public:
    subarm(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    subarmController c;
};

DECLARE_INTERFACE(subarm)

#endif