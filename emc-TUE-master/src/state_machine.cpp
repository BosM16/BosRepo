#include "state_machine.hpp"

bool state_machine(struct ROB_state *rob, struct Event_Queue *eq) {
    // State Machine

    // Parsing Event Queue
    bool too_close_to_wall = false;
    bool no_walls_in_neighbourhood = false;
    bool distance_within_bounds = false;
    bool angle_within_bounds = false;


        while (!eq->events.empty()) {
            Event ev = eq->events.back();
            eq->events.pop_back();

            if (ev == TOO_CLOSE_TO_WALL)
                too_close_to_wall = true;
            if (ev == NO_WALLS_IN_NEIGHBOURHOOD)
                no_walls_in_neighbourhood = true;
            if (ev == DISTANCE_WITHIN_BOUNDS)
                distance_within_bounds = true;
            if (ev == ANGLE_WITHIN_BOUNDS)
                angle_within_bounds = true;
    }



    switch(rob->FSM_state) {
        // case drive_forward: the robot drives forward until a wall is detected.
        case STOP:

            std::cout << "FSM_state = STOP" << std::endl;
            break;
            // case drive_backward: the robot drives backward

        case SEARCH_WALL:
            if(too_close_to_wall)
                rob->FSM_state = STOP;
            std::cout << "FSM_state = SEARCH_WALL" << std::endl;
            break;

        case DRIVE_TO_WALL:
            if(too_close_to_wall) {
                rob->FSM_state = STOP;
            }
            std::cout << "FSM_state = DRIVE_TO_WALL" << std::endl;
//            std::cout << std::endl << "within distance = " << distance_within_bounds << std::endl
//                      << "within angle = " << angle_within_bounds << std::endl << std::endl;
            if (distance_within_bounds && angle_within_bounds){
                std::cout << "IN BOUNDS" << std::endl;
                rob->FSM_state = FOLLOW_WALL;
                rob->desired_robot_state->vx = rob->cp->speed_x;
                rob->desired_robot_state->vy = 0;
                rob->desired_robot_state->vtheta = 0;
                std::cout << "FSM_state = DRIVE_TO_WALL -> FOLLOW_WALL" << std::endl;
            }


            break;
        case FOLLOW_WALL:
            if(too_close_to_wall)
                rob->FSM_state = STOP;
            std::cout << "FSM_state = FOLLOW_WALL" << std::endl;

            break;
        default:
            // stop
            rob->FSM_state = STOP;
            std::cout << "FSM_state = DEFAULT" << std::endl;
            break;
    }
  return true;
}
