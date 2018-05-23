#include "control.hpp"

bool control_follow_wall(struct ROB_state *rob, struct WM_state *wm) {

    // If wall is left, everything is ok. If wall is right, invert signs
    if (rob->cp->side == RIGHT) {
        rob->desired_robot_state->vy = -rob->desired_robot_state->vy;
        rob->desired_robot_state->vtheta = -rob->desired_robot_state->vtheta;

    }

    if (wm->dist_meas->min_distance_measured - rob->cp->desired_distance > rob->cp->tolerance_on_desired_distance) {
        // if measured min distance is larger than tolerancee, give it positive motion in y direction
        rob->desired_robot_state->vy += rob->cp->control_increment_y;
    }else if (wm->dist_meas->min_distance_measured - rob->cp->desired_distance < -rob->cp->tolerance_on_desired_distance) {
        // if measured min distance is smaller than tolerance, give it a negative motion in y direction
        rob->desired_robot_state->vy -= rob->cp->control_increment_y;
    }else {
        if (std::abs(rob->desired_robot_state->vy) > 0.00001) {
            rob->desired_robot_state->vy *= 0.01;
        } else{
            rob->desired_robot_state->vy = 0;
        }
    }

    if (wm->dist_meas->forward_distance_measured * std::cos(rob->cp->angle_between_min_distance_and_forward_check) -
        rob->cp->desired_distance > rob->cp->tolerance_on_desired_distance) {
        // if measured distance ahead is larger than tolerance, compensate with counter clock-wise rotation
        rob->desired_robot_state->vtheta += rob->cp->contol_increment_angle;
    } else if (wm->dist_meas->forward_distance_measured * std::cos(rob->cp->angle_between_min_distance_and_forward_check) -
        rob->cp->desired_distance < -rob->cp->tolerance_on_desired_distance) {
        // if measured distance ahead is smaller than tolerance, compensate with clock-wise rotation
        rob->desired_robot_state->vtheta -= rob->cp->contol_increment_angle;
    } else {
        if (std::abs(rob->desired_robot_state->vtheta) > 0.00001) {
            rob->desired_robot_state->vtheta *= 0.01;
        } else{
            rob->desired_robot_state->vtheta = 0;
        }
    }
    // If wall is left, everything is ok. If wall is right, invert signs back
    if (rob->cp->side == RIGHT) {
        rob->desired_robot_state->vy = -rob->desired_robot_state->vy;
        rob->desired_robot_state->vtheta = -rob->desired_robot_state->vtheta;

    }
    return true;
}

bool control_drive_towards_wall(struct ROB_state *rob, struct WM_state *wm) {

    // If wall is left, everything is ok. If wall is right, invert signs
    if (rob->cp->side == RIGHT)
        rob->desired_robot_state->vtheta = -rob->desired_robot_state->vtheta;

    // move in direction of closest point
    if (wm->dist_meas->min_distance_measured - rob->cp->desired_distance > rob->cp->tolerance_on_desired_distance) {
        rob->desired_robot_state->vx += rob->cp->control_increment_x * std::cos(wm->dist_meas->angle_min_distance_measured);
        rob->desired_robot_state->vy += rob->cp->control_increment_x * std::sin(wm->dist_meas->angle_min_distance_measured);
    } else if (wm->dist_meas->min_distance_measured - rob->cp->desired_distance <
               -rob->cp->tolerance_on_desired_distance) {
        rob->desired_robot_state->vx -= rob->cp->control_increment_x * std::cos(wm->dist_meas->angle_min_distance_measured);
        rob->desired_robot_state->vy -= rob->cp->control_increment_x * std::sin(wm->dist_meas->angle_min_distance_measured);
    } else {
        if (std::abs(rob->desired_robot_state->vx) > 0.00001 && std::abs(rob->desired_robot_state->vy) > 0.00001) {
            rob->desired_robot_state->vx *= 0.1;
            rob->desired_robot_state->vy *= 0.1;
        } else{
            rob->desired_robot_state->vx = 0;
            rob->desired_robot_state->vy = 0;
        }
    }
    // align to closest point
    if (std::abs(wm->dist_meas->angle_min_distance_measured) >
            (90 / 180.0 * 3.14 + rob->cp->tolerance_on_desired_angle)) {
        // angle is larger than tolerance, compensate with counter clock-wise rotation
        rob->desired_robot_state->vtheta += rob->cp->contol_increment_angle;
    } else if (std::abs(wm->dist_meas->angle_min_distance_measured) <
            (90 / 180.0 * 3.14 - rob->cp->tolerance_on_desired_angle)) {
        // angle is larger than tolerance, compensate with clock-wise rotation
        rob->desired_robot_state->vtheta -= rob->cp->contol_increment_angle;
    } else {
        if (std::abs(rob->desired_robot_state->vtheta) > 0.00001) {
            rob->desired_robot_state->vtheta *= 0.1;
        } else{
            rob->desired_robot_state->vtheta = 0;
        }
    }

    // If wall is left, everything is ok. If wall is right, invert signs back
    if (rob->cp->side == RIGHT)
        rob->desired_robot_state->vtheta = -rob->desired_robot_state->vtheta;

    return true;
}


bool control_turn_around_corner() {
    // TODO:Change controller when executing a turn
    return true;
}

bool control(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq) {

    if (rob->FSM_state == STOP) {
        // Compute control inputs
        rob->desired_robot_state->vx = 0;
        rob->desired_robot_state->vy = 0;
        rob->desired_robot_state->vtheta = 0;

    } else if (rob->FSM_state == DRIVE_TO_WALL) {
        // Compute control inputs
        if (!control_drive_towards_wall(rob, wm))
            return false;

    } else if (rob->FSM_state == FOLLOW_WALL) {
        // Compute control inputs
        if (!control_follow_wall(rob, wm))
            return false;

    }

    // Print wall side + velocity reference to console
    if (rob->cp->side == LEFT)
        std::cout << "Left side:";
    else if (rob->cp->side == RIGHT)
        std::cout << "Right side:";
    else
        std::cout << "No side:";
    std::cout << " vx " << rob->desired_robot_state->vx
              << ", vy " << rob->desired_robot_state->vy
              << ", vtheta " << rob->desired_robot_state->vtheta << std::endl;

    // Send velocity reference to base controller
    rob->io.sendBaseReference(rob->desired_robot_state->vx, rob->desired_robot_state->vy, rob->desired_robot_state->vtheta);

    return true;
}
