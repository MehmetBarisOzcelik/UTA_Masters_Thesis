README.txt
====================================================================
Autonomous UAV Mission Planning Under Threat Using Model Predictive
Control with Proportional-Navigation Pursuers
====================================================================

Author:
    Mehmet Barış Özçelik

Program:
    M.S. Aerospace Engineering
    Department of Mechanical and Aerospace Engineering
    The University of Texas at Arlington

Advisor:
    Dr. Kamesh Subbarao

Project Type:
    Master's thesis MATLAB simulation framework


====================================================================
1. Project Overview
====================================================================

This MATLAB project implements a two-dimensional autonomous UAV mission
planning and guidance simulation in a contested environment. The UAV
must travel from a start location to a goal while visiting required
checkpoints and avoiding no-fly zones (NFZs) and radar threat regions.
A chaser attempts to intercept the UAV using either a basic
pure-pursuit-style guidance law or a proportional-navigation guidance
law.

The project compares:

    1. Basic pursuit vs proportional navigation
    2. Non-MPC route-following vs MPC-based evasive guidance
    3. UAV survivability under different chaser-guidance combinations
    4. Effects of proportional-navigation gain, chaser-to-UAV speed
       ratio, and MPC horizon length
    5. Safety metrics such as NFZ clearance, radar clearance, exposure,
       violations, miss distance, catch flag, and computation time

The main goal is to provide a modular MATLAB-based simulation and
presentation framework for analyzing UAV guidance, threat avoidance,
and pursuit-evasion behavior under consistent and fair assumptions.


====================================================================
2. Main Thesis Objective
====================================================================

The main objective of the thesis is to evaluate whether model predictive
control improves UAV survivability in a threat environment and whether
proportional navigation improves chaser effectiveness compared with a
basic pursuit law.

The main full-mission comparison is intentionally fair:

    - Same environment
    - Same chaser initial condition
    - Same chaser-to-UAV speed ratio
    - Same chaser turn-rate limit
    - Same capture radius
    - Only the guidance law changes between the basic and
      proportional-navigation chasers

The four main full-mission cases are:

    1. route + basic
    2. route + PN
    3. MPC   + basic
    4. MPC   + PN

This structure supports the main thesis conclusions:

    - PN is more effective than basic pursuit because it catches faster
      or reduces the miss distance under the same physical limits.

    - MPC improves UAV survivability because it generates safer
      constraint-aware trajectories compared with simple route-following.

    - PN remains more dangerous than basic pursuit under MPC because it
      produces a smaller miss distance, even when capture is avoided.


====================================================================
3. Folder Structure
====================================================================

The project is organized as follows:

    core/
        Main simulation and control logic.
        Important files:
            run_all.m
            advance_states.m
            mpc_step.m
            simple_route_step.m
            waypoint_manager.m

    demo/
        Thesis demonstration scripts and parameter sweeps.
        Important files:
            sim1_pn_vs_pp_crossing.m
            sim1_pn_vs_pp_crossing_anim.m
            sim2_mpc_avoid.m
            sim2_mpc_avoid_anim.m
            sim3_full_mission_compare.m
            sim3_param_sweep.m

    env/
        Environment generation, routing, and graph construction.
        Important files:
            config.m
            threat_map.m
            manual_env_input.m
            mission_presets.m
            build_graph.m
            route_optimizer.m
            segment_circle_intersect.m

    plotting/
        Static plotting and animation utilities.
        Important files:
            plot_mission.m
            animate_mission.m

    pursuer/
        Chaser initialization and guidance models.
        Important files:
            init_pursuer.m
            simulate_pursuer_basic.m
            simulate_pursuer_pn.m
            miss_distance.m

    utils/
        Small helper utilities.
        Important files:
            wrapToPi.m

    run_thesis_menu.m
        Main GUI launcher for the thesis simulations.

    run_thesis_gui.m
        Simple GUI launcher alternative.

    README.txt
        This file.


====================================================================
4. How to Run the Project
====================================================================

Open MATLAB and set the current folder to the main project folder.

Then run:

    clear; clc; close all
    addpath(genpath(pwd))
    run_thesis_menu

This opens the main thesis GUI. From the GUI, the user can select and
run the main demonstrations.

The main GUI is the recommended way to present the thesis simulations
during a defense or project demonstration.


====================================================================
5. Main GUI Demonstrations
====================================================================

The GUI provides four main demonstrations.


--------------------------------------------------------------------
Demo 1: PN vs Pure Pursuit Crossing Engagement
--------------------------------------------------------------------

Files:
    demo/sim1_pn_vs_pp_crossing.m
    demo/sim1_pn_vs_pp_crossing_anim.m

Purpose:
    Runs the main full-mission scenario with a UAV, checkpoints,
    NFZs, radar zones, and an active chaser.

Fairness:
    Both interceptors use:
        - same initial position
        - same speed
        - same acceleration/turning limit
        - same target trajectory

    The only difference is the guidance law.

Expected result:
    PN intercepts the target faster and with a shorter path, while pure
    pursuit either misses or produces a longer, less efficient trajectory.

Main metrics:
    - intercept time
    - minimum separation
    - path length


--------------------------------------------------------------------
Demo 2: MPC vs No-MPC NFZ Avoidance
--------------------------------------------------------------------

Files:
    demo/sim2_mpc_avoid.m
    demo/sim2_mpc_avoid_anim.m

Purpose:
    Demonstrates the benefit of MPC-based obstacle avoidance in a simple
    environment with two no-fly zones.

Fairness:
    Both UAV cases use:
        - same start
        - same goal
        - same vehicle model
        - same speed limits
        - same NFZ geometry

    The only difference is the guidance strategy.

Cases:
    1. No-MPC go-to-goal guidance
    2. MPC-based avoidance guidance

Expected result:
    The no-MPC trajectory cuts through or passes dangerously close to an
    NFZ, while the MPC trajectory bends around the NFZs and maintains
    positive clearance.

Main metrics:
    - final time
    - path length
    - minimum NFZ clearance


--------------------------------------------------------------------
Demo 3: Fair Full-Mission Comparison
--------------------------------------------------------------------

File:
    demo/sim3_full_mission_compare.m

Purpose:
    Runs the main full-mission scenario with a UAV, checkpoints,
    NFZs, radar zones, and a chaser.

Cases:
    1. Default environment + route guidance + basic chaser
    2. Default environment + route guidance + proportional-navigation chaser
    3. Default environment + MPC guidance   + basic chaser
    4. Default environment + MPC guidance   + proportional-navigation chaser
    5. Manual environment with user-selected UAV and chaser guidance

Fairness:
    In cases 1 through 4, the basic and proportional-navigation chasers
    use the same physical parameters. The only difference is the chaser
    guidance law.

Expected result:
    The route-following UAV avoids the basic chaser but is captured by
    the proportional-navigation chaser. The MPC-guided UAV avoids capture
    against both chaser types. Proportional navigation remains more
    dangerous under MPC because it produces a smaller miss distance than
    the basic chaser under the same physical limits.

Main metrics:
    - final time
    - catch flag
    - miss distance
    - path length
    - NFZ clearance
    - radar clearance
    - exposure metrics


--------------------------------------------------------------------
Demo 4: Parameter Sweeps and Mission Statistics
--------------------------------------------------------------------

File:
    demo/sim3_param_sweep.m

Purpose:
    Runs quantitative studies to support the thesis results.

Studies:
    A. Fair four-case comparison
    B. PN navigation constant sweep
    C. Chaser-to-UAV speed-ratio sweep
    D. MPC horizon sweep

The route risk-weight sweep is intentionally not included in the main
results because, for the current default environment, changing the
risk weight does not change the selected route. The routing algorithm
still includes risk-aware cost capability, but this parameter is not
emphasized as a main result.

Output workspace tables:
    T_fair
    T_pnGain
    T_speed
    T_horizon

Main metrics:
    - final time
    - catch flag
    - miss distance
    - path length
    - minimum NFZ clearance
    - minimum radar clearance
    - NFZ violation count
    - radar violation count
    - radar exposure integral
    - total exposure integral
    - mean MPC computation time
    - maximum MPC computation time


====================================================================
6. Important MATLAB Commands
====================================================================

Run the main GUI:

    run_thesis_menu

Run the simpler GUI:

    run_thesis_gui

Run Demo 1 static plot:

    sim1_pn_vs_pp_crossing

Run Demo 1 animation:

    sim1_pn_vs_pp_crossing_anim

Run Demo 2 static plot:

    sim2_mpc_avoid

Run Demo 2 animation:

    sim2_mpc_avoid_anim

Run Demo 3, case 4, plot only:

    sim3_full_mission_compare(4,1)

Run Demo 3, case 4, plot + animation:

    sim3_full_mission_compare(4,2)

Run Demo 4 parameter sweeps:

    sim3_param_sweep


====================================================================
7. Configuration
====================================================================

The main configuration file is:

    env/config.m

This file defines:

    - simulation time step and maximum time
    - environment bounds and threat-zone parameters
    - UAV speed, acceleration, and turn-rate limits
    - route-following gains
    - chaser-to-UAV speed ratio, PN gain, and chaser turn-rate limits
    - MPC horizon, control grids, and cost weights
    - plotting and output options
    - catch radius and metric settings

Most thesis parameters should be changed in config.m rather than inside
individual demo files.


====================================================================
8. Environment and Threat Modeling
====================================================================

The environment is created by:

    env/threat_map.m

The environment contains:

    - start point
    - goal point
    - checkpoints
    - no-fly zones
    - radar zones
    - map bounds

NFZs and radars are modeled internally as circular disks for distance
and collision calculations. In plots, NFZs are drawn as hexagons to make
them visually distinct from radar zones.

The environment generator checks:

    - start, checkpoints, and goal are outside threat disks
    - NFZs do not overlap each other
    - radar zones do not overlap each other
    - NFZs and radars do not overlap each other
    - mission points are not too close to threat boundaries


====================================================================
9. Routing
====================================================================

Routing is handled by:

    env/build_graph.m
    env/route_optimizer.m

The planner builds an augmented visibility graph using:

    - required mission nodes:
        start, checkpoints, goal

    - auxiliary detour nodes:
        automatically generated around NFZ/radar boundaries

Edges are rejected if they intersect keep-out regions. Feasible edges
are assigned a cost based on distance and threat/risk exposure.

The route optimizer then finds a feasible path through all required
mission points. This avoids relying on a simple fallback route and makes
the full-mission simulation cleaner and more defensible.


====================================================================
10. UAV Guidance
====================================================================

Two UAV guidance modes are implemented.

--------------------------------------------------------------------
Route mode
--------------------------------------------------------------------

File:
    core/simple_route_step.m

Description:
    Uses proportional heading control toward the current waypoint.
    This mode does not explicitly reason about NFZs or radar regions and
    acts as the non-MPC baseline.

--------------------------------------------------------------------
MPC mode
--------------------------------------------------------------------

File:
    core/mpc_step.m

Description:
    Uses a receding-horizon grid-search MPC controller. The controller
    evaluates candidate heading-rate and acceleration commands and selects
    the command that minimizes a cost containing:

        - waypoint tracking error
        - speed tracking error
        - NFZ proximity penalty
        - radar proximity penalty
        - hard penalty for entering keep-out zones
        - control effort penalty

Only the first control command is applied at each time step, and the
optimization is repeated at the next step.


====================================================================
11. Chaser Models
====================================================================

The chaser has two guidance modes.

--------------------------------------------------------------------
Basic chaser
--------------------------------------------------------------------

File:
    pursuer/simulate_pursuer_basic.m

Description:
    A pure-pursuit style chaser that turns directly toward the current
    UAV position using a proportional heading-error controller.

--------------------------------------------------------------------
Proportional-navigation chaser
--------------------------------------------------------------------

File:
    pursuer/simulate_pursuer_pn.m

Description:
    A proportional-navigation chaser that uses line-of-sight rate to
    guide toward the UAV. This represents a more capable interceptor.

Fairness note:
    In the main comparison, the basic and proportional-navigation chasers use the same speed,
    start position, turn-rate limit, and capture radius. Only the guidance law changes.


====================================================================
12. Metrics
====================================================================

The simulation computes the following metrics:

Interception metrics:
    - catch flag
    - catch index
    - catch position
    - miss distance

Mission metrics:
    - final time
    - UAV path length
    - checkpoint/goal completion through waypoint manager

Safety metrics:
    - minimum NFZ clearance
    - minimum radar clearance
    - NFZ violation count
    - radar violation count

Exposure metrics:
    - NFZ exposure integral
    - radar exposure integral
    - total exposure integral

Computation metrics:
    - mean MPC computation time
    - maximum MPC computation time
    - number of MPC steps


====================================================================
13. Typical Thesis Results
====================================================================

The current fair full-mission comparison gives the following qualitative
behavior:

    route + basic:
        UAV avoids capture by the basic chaser, but with a relatively
        small miss distance.

    route + PN:
        UAV is captured by the proportional-navigation chaser under the same physical pursuer
        limits. This demonstrates the increased effectiveness of the PN
        guidance law compared with basic pursuit.

    MPC + basic:
        UAV avoids capture and reaches the goal with a larger miss
        distance than the route + basic case.

    MPC + PN:
        UAV avoids capture and reaches the goal. The miss distance is
        smaller than in the MPC + basic case, showing that PN remains
        more dangerous even when MPC guidance is used.

This supports the thesis conclusions:

    1. PN is more effective than basic pursuit because PN captures the
       route-following UAV while the basic chaser does not under the
       same physical limits.

    2. MPC improves UAV survivability because the UAV avoids capture
       against both basic and proportional-navigation chasers.

    3. PN remains more dangerous than basic pursuit under MPC because it
       produces a smaller miss distance than the basic pursuer.

    4. Longer MPC horizons can improve safety-related metrics, but they
       increase computation time.

    5. Increasing chaser-to-UAV speed ratio makes interception more likely and
       generally reduces catch time.

====================================================================
14. GUI Role in the Thesis
====================================================================

The GUI is an important part of the project. It is not only a convenience
tool; it is also a demonstration and reproducibility platform.

The GUI allows a user or thesis committee member to run the major
simulation cases without manually editing code. This makes the simulation
framework easier to present, test, and reproduce.

Recommended defense structure:

    1. Use slides to explain the problem, models, equations, and metrics.
    2. Use the GUI to run the main simulations live.
    3. Use saved figures/tables as backup in case live demos are slow.
    4. Use parameter-sweep tables to support numerical conclusions.


====================================================================
15. Notes and Limitations
====================================================================

This project uses a simplified two-dimensional point-mass UAV model.
The current simulation assumes:

    - planar motion
    - one UAV
    - one chaser
    - known states
    - no sensor noise
    - no wind
    - no actuator dynamics beyond simple limits
    - circular internal threat regions

These assumptions are acceptable for the current thesis scope because
the focus is on guidance comparison, threat avoidance, and mission-level
simulation. Future work may include:

    - 3-D vehicle dynamics
    - multiple chasers
    - multiple UAVs
    - state estimation and sensor uncertainty
    - wind and disturbances
    - ROS 2 implementation
    - higher-fidelity MPC solvers
    - machine-learning-assisted mission planning


====================================================================
16. Troubleshooting
====================================================================

If MATLAB cannot find functions, run:

    addpath(genpath(pwd))

If the GUI does not open, verify that the current MATLAB folder is the
main project folder.

If animations are slow, use plot-only mode.

If figures or old variables interfere with a run, use:

    clear; clc; close all

If parameter sweeps take time, this is expected because each sweep runs
multiple full simulations.

If route planning appears incorrect, check:

    env/build_graph.m
    env/route_optimizer.m
    env/threat_map.m

If chaser behavior appears incorrect, check:

    pursuer/simulate_pursuer_basic.m
    pursuer/simulate_pursuer_pn.m
    pursuer/init_pursuer.m

====================================================================
17. Recommended Thesis Figures
====================================================================

Recommended figures for the thesis document:

    Figure 1:
        GUI screenshot

    Figure 2:
        Project folder/module structure

    Figure 3:
        Demo 1: PN vs pure pursuit crossing target

    Figure 4:
        Demo 2: MPC vs no-MPC NFZ avoidance

    Figure 5:
        Full mission, route + PN

    Figure 6:
        Full mission, MPC + PN

    Figure 7:
        Fair four-case comparison table/bar chart

    Figure 8:
        PN gain sweep: catch time vs PN gain

    Figure 9:
        Speed-ratio sweep: capture time vs chaser-to-UAV speed ratio

    Figure 10:
        MPC horizon sweep: safety metric and computation time vs horizon


====================================================================
18. End of README
====================================================================