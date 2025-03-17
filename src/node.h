#ifndef __NODE_AGENT__H
#define __NODE_AGENT__H 


#include "enviro.h"

using namespace enviro;

class nodeController : public Process, public AgentInterface {

    public:
    nodeController() : Process(), AgentInterface() {}

    void init() {
        watch("head_pos", [this](Event &e) {
            double head_x = e.value()["x"];
            double head_y = e.value()["y"];
    
            double distance = (head_x - x()) * (head_x - x()) + (head_y - y()) * (head_y - y());
            if (distance <= 25.0) {
                emit(Event("contacted", 
                    json({{"id", id()}}))
                );
                remove_agent(id());   
            }
        });
    }

    void start() {}
    void update() {
        
    }
    void stop() {}

};

class node : public Agent {
    public:
    node(json spec, World& world) : Agent(spec, world) {
        add_process(c);
    }
    private:
    nodeController c;
};

DECLARE_INTERFACE(node)

#endif