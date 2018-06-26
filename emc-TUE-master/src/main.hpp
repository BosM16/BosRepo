
#ifndef MAIN_HPP_INCLUDED
#define MAIN_HPP_INCLUDED

#include <emc/io.h>
#include <emc/rate.h>

#include "perception.hpp"
#include "monitors.hpp"
#include "state_machine.hpp"
#include "control.hpp"
#include "helper.hpp"

#define PI  3.14159265359

typedef enum event{
    TOO_CLOSE_TO_WALL,
    NO_WALLS_IN_NEIGHBOURHOOD,
    DISTANCE_WITHIN_BOUNDS,
    ANGLE_WITHIN_BOUNDS
} Event;

struct Event_Queue {
    //TODO: define
    std::vector<Event> events;
};

typedef enum state {
    STOP,
    SEARCH_WALL,
    DRIVE_TO_WALL,
    FOLLOW_WALL
} State;

enum Side {
    LEFT = 0,
    RIGHT = 1,
    NONE = 2
};

typedef struct Control_Parameters_Follow_Wall {
    float desired_distance;
    float tolerance_on_desired_distance;
    float tolerance_on_desired_angle;
    float angle_between_min_distance_and_forward_check;
    Side side; // do we follow a wall left or right of the robot
    float speed_x;
    float control_increment_x; // needed for drive towards wall
    float control_increment_y;
    float contol_increment_angle;
    float K_y;
    float K_theta;
} Control_Parameters_Follow_Wall;

typedef struct Perception_Parameters_Drive_Towards_Wall {
    float local_neighbourhood;
    float tolerance_extrema;
    float length_wall_segment_considered;

    // Interval of distances considered during wall following
    Interval int_left_side;
    Interval int_right_side;
    Interval int_left_forward;
    Interval int_right_forward;

    //TODO added for filtered_error_measurement_at_angle
    float corner_search_angle;
    float [3] error_at_angle;
    float [3] filtered_error_at_angle;
    float Ts;
    float fc;
} Perception_Parameters_Drive_Towards_Wall;

typedef struct Monitor_Parameters_Drive_Towards_Wall {
    float close_to_wall;
} Monitor_Parameters_Drive_Towards_Wall;

typedef struct Distance_Measurement {
    std::vector<float> *distances;
    float angle_min_distance_measured;
    float min_distance_measured;
    float forward_distance_measured;
} Distance_Measurement;

typedef struct WM_state {
    Distance_Measurement *dist_meas;
    Wall_Follow_Angle_Error *wall_follow_angle_error;
} WM_state;

typedef struct Desired_Robot_State {
    float vx;
    float vy;
    float vtheta;
} Desired_Robot_State;

typedef struct ROB_state {
    emc::IO io;
    emc::Rate r;
    emc::LaserData scan;
    emc::OdometryData odom;
    State FSM_state;
    Desired_Robot_State *desired_robot_state;
    Control_Parameters_Follow_Wall *cp;
    Perception_Parameters_Drive_Towards_Wall *pp;
    Monitor_Parameters_Drive_Towards_Wall *mp;

    ROB_state(int rate, Control_Parameters_Follow_Wall *cp, Perception_Parameters_Drive_Towards_Wall *pp,
              Monitor_Parameters_Drive_Towards_Wall *mp, State FSM_state, Desired_Robot_State *desired_robot_state)
            : io(), r(rate), cp(cp), pp(pp), mp(mp), FSM_state(FSM_state), desired_robot_state(desired_robot_state) {}

    ROB_state() : io(), r(10), FSM_state(DRIVE_TO_WALL) {}
} ROB_state;

void visualization(const std::vector<Point> &pts);


#endif
