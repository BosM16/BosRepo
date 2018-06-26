#include "monitors.hpp"


bool monitor_drive_towards_wall(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq) {

    // Monitor if the robot exceeds the minimum allowed distance
    for (std::vector<float>::iterator it = (*(wm->dist_meas)->distances).begin();
         it != (*(wm->dist_meas)->distances).end(); ++it) {
        if (*it < rob->mp->close_to_wall) {
            eq->events.push_back(TOO_CLOSE_TO_WALL); //event
        }

    }

    // Monitor whether robot reaches the desired distance within tolerance
    if ((wm->dist_meas->min_distance_measured - rob->cp->desired_distance < rob->cp->tolerance_on_desired_distance &&
         wm->dist_meas->min_distance_measured - rob->cp->desired_distance > -rob->cp->tolerance_on_desired_distance)) {
        eq->events.push_back(DISTANCE_WITHIN_BOUNDS);
    }

    // Monitor whether robot is aligned to wall within tolerance
    if ((std::abs(wm->dist_meas->angle_min_distance_measured) - 0.5 * PI <
         rob->cp->tolerance_on_desired_angle &&
         std::abs(wm->dist_meas->angle_min_distance_measured) - 0.5 * PI >
         -rob->cp->tolerance_on_desired_angle)) {
        eq->events.push_back(ANGLE_WITHIN_BOUNDS);
    }
    return true;

}

bool monitor_follow_wall(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq) {
    // TODO: Implement your monitors in the wall following case (minimum distance, wall in front, open space in side wall ...)
    return true;
}

bool monitors(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq) {

    *(wm->dist_meas)->distances;


    if (rob->FSM_state == DRIVE_TO_WALL) {

        if (!monitor_drive_towards_wall(rob, wm, eq))
            return false;
    }

    if (rob->FSM_state == FOLLOW_WALL) {

        if (!monitor_follow_wall(rob, wm, eq))
            return false;
    }

    return true;
}
