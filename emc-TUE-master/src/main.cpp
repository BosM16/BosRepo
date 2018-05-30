#include "main.hpp"

int main() {

    //Initialize control Control_Parameters
    Control_Parameters_Follow_Wall cp;
    cp.desired_distance = 0.3; //in [m]
    cp.tolerance_on_desired_distance = 0.05; //in [m]
    cp.tolerance_on_desired_angle = 5 * PI / 180.0; //in  [rad]
    cp.angle_between_min_distance_and_forward_check = std::sqrt(2) / 2; // in [rad]
    cp.side = LEFT;
    cp.speed_x = 0.13; // in [m/s]
    cp.control_increment_x = 0.02; // in [m/s]
    cp.control_increment_y = 0.03; // in [m/s]
    cp.contol_increment_angle = 2 * (2 * PI) / 360; //in [rad]
    cp.K_y = 1.0;
    cp.K_theta = 1.0;

    //Initialize perception Perception_Parameters
    Perception_Parameters_Drive_Towards_Wall pp;
    pp.local_neighbourhood = 10;// [m]
    pp.tolerance_extrema = 0.02; // [m]
    pp.length_wall_segment_considered = 0.1; // [m]
    pp.int_left_side = Interval(0,0);
    pp.int_right_side = Interval(0,0);
    pp.int_left_forward = Interval(0,0);
    pp.int_right_forward = Interval(0,0);


    //Initialize perception Perception_Parameters
    Monitor_Parameters_Drive_Towards_Wall mp;
    mp.close_to_wall = 0.1;// [m]

    // Initialization of world model
    WM_state wm;

    Distance_Measurement dist_meas;
    std::vector<float> distances;
    wm.dist_meas = &dist_meas;
    dist_meas.distances = &distances;
    dist_meas.forward_distance_measured = 0;
    dist_meas.forward_distance_measured = 0;

    // Initialization of event queue
    Event_Queue eq;

    // Initialization of the FSM state
    State fsm_state = DRIVE_TO_WALL;

    // Initialization of robot motion state
    Desired_Robot_State desired_robot_state;
    desired_robot_state.vx = 0; 
    desired_robot_state.vy = 0;
    desired_robot_state.vtheta = 0;

    // Initialization of robot
    ROB_state rob(10, &cp, &pp, &mp, fsm_state, &desired_robot_state);

    // initialization of wall following perception
    initialize_wall_following(&rob,&wm);

    // Loop while we are properly connected
    while (rob.io.ok()) {
        // EVENT LOOP

        if(!perception(&rob, &wm)){
            rob.r.sleep();
            std::cout << "looping";
            continue;

        }

        monitors(&rob, &wm,&eq);
        state_machine(&rob,&eq);
        control(&rob, &wm, &eq);

        // Sleep remaining time
        rob.r.sleep();
    }

    return 0;

}
