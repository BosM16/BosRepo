# BosRepo

<img src='https://g.gravizo.com/svg?;

@startuml;

participant "Bridge" as A;
participant "Motionplanner" as B;
participant "ObstDetect" as C;

User -> A: INIT - req:global_costmap;
activate A;
A -> C: process_global_costmap/req: global_costmap;
activate C;
C -> A: resp: basic shapes room, static obstacles;
A -> B: room, static obstacles;
activate B;
deactivate B;
A -> User: Resp: None;
Deactivate A;

@enduml;

'>
