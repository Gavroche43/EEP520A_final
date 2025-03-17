# EEP520A_final
EEP520_final

# 3-Segment Robotic Arm Simulator

## Overview
This project is a 3-segment robotic arm simulator that implements the Cyclic Coordinate Descent (CCD) algorithm for inverse kinematics. The goal is to simulate the motion of a multi-jointed robotic arm to reach a given target position in a 2D plane.

## Key Challenges and Solutions
One of the major challenges faced during development was designing the simulation architecture. The `enviro` environment is not inherently suited for multi-segmented agents, making the initial approach—where different segments and joints were treated as individual agents—problematic due to collision mechanics. To overcome this:
- The collision handler in `enviro` was ignored.
- A custom collision detection and handling mechanism was implemented.
- The robot was structured as a single static agent rather than multiple independent agents.

This restructuring allowed for smoother inverse kinematics computations and better control over the robotic arm's movements.

## Installation and Setup
To install and run the project, follow these steps:

1. Ensure you have Docker installed on your system.
2. Run the `enviro` Docker container:
   ```sh
   docker run -p80:80 -p8765:8765 -v ${PWD}:/source -it klavins/enviro:v1.61 bash
   ```

## Running the Simulation
Once the Docker container is running, follow these steps to start the simulation:

1. In the terminal, start the `enviro` environment:
   ```sh
   esm start
   ```
2. Launch `enviro`:
   ```sh
   enviro
   ```
3. The simulation should now be running in your browser. The robotic arm will attempt to move its end-effector to the target position using the CCD algorithm.

## Acknowledgments
This project was developed using external resources for reference and guidance:
- **Inverse Kinematics Algorithm:** [Stack Overflow](https://stackoverflow.com/questions/24095107/working-of-ccd-algorithm-for-inverse-kinematics)
- **Doxygen Comments Assistance:** GPT-4.5 was used to generate structured documentation for the codebase.

## License
This project is open-source and free to use for educational and research purposes.

