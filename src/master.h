#ifndef __MASTER_AGENT__H
#define __MASTER_AGENT__H 

#include "enviro.h"
#include <cmath>   
#include <cstdlib>    
#include <ctime>     
#include <string>

using namespace enviro;

class masterController : public Process, public AgentInterface {

    public:
        masterController() : Process(), AgentInterface() {
            // Seed the random number generator.
            std::srand(std::time(nullptr));
        }

        /// Target angles for the joints (in radians)
        double joint_1_theta_target = 0;
        double joint_2_theta_target = 0;
        double joint_3_theta_target = 0;
        
        /// Current angles for the joints (in radians)
        double joint_1_theta = 0;
        double joint_2_theta = 0;
        double joint_3_theta = 0;
        
        /// Target node (position) coordinates
        double target_x = -30;
        double target_y = 45;

        bool spawn_decision = true;
        
        void init() {}

        void start() {
            nodes_spawned = 0;
            watch("contacted", [this](Event e) {
                if (!agent_exists(e.value()["id"])) {
                    spawn_decision = true;
                }
            });
        }
        
        void update() {
            updateInverseKinematics();
            updateJointAngles(0.05);
            emitJointEvents();
            if (spawn_decision) {
                spawn_decision = false;
                spawnNewNode();
            }
        }
        
        void stop() {}

    private:
        int nodes_spawned = 0;
        const int max_nodes = 33;

        /**
         * @brief Helper function to compute the signed angle between two vectors.
         *
         * Returns the angle in radians by which you need to rotate vector (v1x, v1y) to align with (v2x, v2y).
         */
        double angleBetweenVectors(double v1x, double v1y, double v2x, double v2y) {
            double dot = v1x * v2x + v1y * v2y;
            double det = v1x * v2y - v1y * v2x;
            return std::atan2(det, dot);
        }
        
        /**
         * @brief Implements the CCD algorithm for a three-link arm.
         *
         * Iteratively updates temporary joint angles (theta1, theta2, theta3) by moving the
         * end-effector closer to the target. The updated angles are then assigned as target angles.
         */
        void updateInverseKinematics() {
            const double L1 = 50.0;
            const double L2 = 50.0;
            const double L3 = 50.0;
            
            const int max_iter = 10;      // Maximum CCD iterations per update
            const double threshold = 1.0; // Acceptable error in position

            // Start with current joint angles.
            double theta1 = joint_1_theta;
            double theta2 = joint_2_theta;
            double theta3 = joint_3_theta;

            for (int iter = 0; iter < max_iter; iter++) {
                // Compute forward kinematics.
                double x1 = L1 * cos(theta1);
                double y1 = L1 * sin(theta1);
                double x2 = x1 + L2 * cos(theta1 + theta2);
                double y2 = y1 + L2 * sin(theta1 + theta2);
                double x3 = x2 + L3 * cos(theta1 + theta2 + theta3);
                double y3 = y2 + L3 * sin(theta1 + theta2 + theta3);
                
                double error = sqrt((target_x - x3)*(target_x - x3) + (target_y - y3)*(target_y - y3));
                if (error < threshold) break;
                
                // --- Joint 3 update ---
                // Pivot point for joint3 is at (x2, y2)
                double v1x = x3 - x2;
                double v1y = y3 - y2;
                double v2x = target_x - x2;
                double v2y = target_y - y2;
                double angle3 = angleBetweenVectors(v1x, v1y, v2x, v2y);
                theta3 += angle3;
                
                // Recompute positions after joint3 update.
                x3 = x2 + L3 * cos(theta1 + theta2 + theta3);
                y3 = y2 + L3 * sin(theta1 + theta2 + theta3);
                error = sqrt((target_x - x3)*(target_x - x3) + (target_y - y3)*(target_y - y3));
                if (error < threshold) break;
                
                // --- Joint 2 update ---
                // Pivot point for joint2 is at (x1, y1)
                v1x = x3 - x1;
                v1y = y3 - y1;
                v2x = target_x - x1;
                v2y = target_y - y1;
                double angle2 = angleBetweenVectors(v1x, v1y, v2x, v2y);
                theta2 += angle2;
                
                // Recompute positions after joint2 update.
                x2 = x1 + L2 * cos(theta1 + theta2);
                y2 = y1 + L2 * sin(theta1 + theta2);
                x3 = x2 + L3 * cos(theta1 + theta2 + theta3);
                y3 = y2 + L3 * sin(theta1 + theta2 + theta3);
                error = sqrt((target_x - x3)*(target_x - x3) + (target_y - y3)*(target_y - y3));
                if (error < threshold) break;
                
                // --- Joint 1 update ---
                // Pivot at the base (0, 0)
                v1x = x3;
                v1y = y3;
                v2x = target_x;
                v2y = target_y;
                double angle1 = angleBetweenVectors(v1x, v1y, v2x, v2y);
                theta1 += angle1;
            }
            
            // Set computed angles as new targets.
            joint_1_theta_target = theta1;
            joint_2_theta_target = theta2;
            joint_3_theta_target = theta3;
        }
        
        /**
         * @brief Gradually moves current joint angles toward target angles.
         *
         * @param constant_speed The fixed angular increment per update.
         */
        void updateJointAngles(double constant_speed) {
            double diff1 = joint_1_theta_target - joint_1_theta;
            if (fabs(diff1) > constant_speed) {
                joint_1_theta += (diff1 > 0 ? constant_speed : -constant_speed);
            } else {
                joint_1_theta = joint_1_theta_target;
            }
            
            double diff2 = joint_2_theta_target - joint_2_theta;
            if (fabs(diff2) > constant_speed) {
                joint_2_theta += (diff2 > 0 ? constant_speed : -constant_speed);
            } else {
                joint_2_theta = joint_2_theta_target;
            }
            
            double diff3 = joint_3_theta_target - joint_3_theta;
            if (fabs(diff3) > constant_speed) {
                joint_3_theta += (diff3 > 0 ? constant_speed : -constant_speed);
            } else {
                joint_3_theta = joint_3_theta_target;
            }
        }
        
        /**
         * @brief Emits events to rotate the joints.
         *
         * Sends events to update the rotation of joint1, joint2, and joint3.
         */
        void emitJointEvents() {
            emit(Event("rotate_joint1", json{{"theta", joint_1_theta}}));
            emit(Event("rotate_joint2", json{{"theta", joint_2_theta}}));
            emit(Event("rotate_joint3", json{{"theta", joint_3_theta}}));
        }
        
        /**
         * @brief Spawns a new node at a random reachable position.
         *
         * If the maximum number of nodes has not been reached, this method generates new 
         * target coordinates within a circle of radius 100, spawns a new node at that location,
         * and increments the node counter.
         */
        void spawnNewNode() {
            if (nodes_spawned < max_nodes) {
                double angle = ((double) std::rand() / RAND_MAX) * 2 * M_PI;
                double radius = ((double) std::rand() / RAND_MAX) * 100;
                target_x = radius * cos(angle);
                target_y = radius * sin(angle);
                
                add_agent("node", target_x, target_y, 0, json{{"fill", "green"}, {"stroke", "black"}});
                nodes_spawned++;
            }
        }
};

class master : public Agent {
    public:
        master(json spec, World& world) : Agent(spec, world) {
            add_process(c);
        }
    private:
        masterController c;
};

DECLARE_INTERFACE(master)

#endif
