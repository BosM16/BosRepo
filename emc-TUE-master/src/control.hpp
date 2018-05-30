#ifndef CONTROLS_HPP_INCLUDED
#define CONTROLS_HPP_INCLUDED

#include "main.hpp"
#include "helper.hpp"

bool control(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq);
bool control_follow_wall(struct ROB_state *rob, struct WM_state *wm);
bool control_drive_towards_wall(struct ROB_state *rob, struct WM_state *wm);
bool control_turn_around_corner(); //not necessary in current design; just here as an example.
float K_y; //gain y velocity
float K_theta; //gain theta velocity

#endif
