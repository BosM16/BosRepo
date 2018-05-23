#ifndef MONITORS_HPP_INCLUDED
#define MONITORS_HPP_INCLUDED

#include "main.hpp"


bool monitor_drive_towards_wall(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq);
bool monitor_follow_wall(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq);
bool monitors(struct ROB_state *rob, struct WM_state *wm, struct Event_Queue *eq) ;

#endif