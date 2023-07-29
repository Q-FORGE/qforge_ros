# QForge ROS

The QForge ROS package contains the code developed by the QForge Team for the 2022 ICUAS UAV competition. Where the team placed second.

The ICUAS UAV competition began with an initial simulation phase which was followed by an experimental phase at the ICUAS venue in Dubrovnik, Croatia. The codebase between the two phases was largely the same, with only minor changes needed due to the different competition arena used for the in-person event.

![QForge autonomous ball drop at ICUAS 2022](/resources/images/icuasuav.gif)

## Running the full solution
To test out the QForge ROS solution, you can clone [The QForge solution repo](https://github.com/Q-FORGE/QFORGE_icuas22) and follow the steps listed in the README. This ROS package will be installed automatically and run on startup.

## Competition objective
The objective of the competition is listed below and must be completed on every run
1. Navigate the UAV through an unknown obstacle grid
2. Search the obstacle free zone for an AR tag placed at an unknown location
3. Publish the location of the AR tag in the world frame
4. Generate a trajectory for the UAV so that it can release a ball (held under the UAV with an electromagnet) such that it impacts AR tag.


## Solution components
The solution is split up into eight components which all contribute to completing the objective. Each component is detailed below.
- State-machine
    - Main contributor: @JadWehbeh
    - Launch: state_machine.launch
    - Objective: Manager of the entire solution. Controls the solution subsystem including which ones are active during each phase of the challenge.

- Pathfinder
    - Main contributor: @15jgme
    - Launch: pathfinder.launch
    - Objective: Compute a collision free path to lead the UAV out of the obstacle zone.

- Ball launch
    - Main contributor: @zhzhuo
    - Launch: ball_launch.launch
    - Objective: Provide a trajectory and release point for the ball so that it impacts the AR tag.

- Search routine
    - Main contributor: @jchernanr24
    - Launch: search_routine.launch
    - Objective: Compute a trajectory to search for the AR tag in the final zone of the challenge.

- AR Tag
    - Main contributor: @15jgme
    - Launch: ar_tag.launch
    - Objective: Detect the AR tag from RBG video data on the UAV and estimate the position of the AR tag in the world frame.

- Navigator
    - Main contributor: @JadWehbeh
    - Launch: navigator.launch
    - Objective: Manages the final position reference for the UAV controller depending on the state of the state-machine.

