# BosRepo

<img src='https://g.gravizo.com/svg?
@startuml;

actor User;
participant "First Class" as A;
participant "Second Class" as B;
participant "Last Class" as C;

User -> A: DoWork;
activate A;

A -> B: Create Request;
activate B;

B -> C: DoWork;
activate C;

C --> B: WorkDone;
destroy C;

B --> A: Request Created;
deactivate B;

A --> User: Done;
deactivate A;

@enduml
'>

<!-- ![Alt text](https://g.gravizo.com/svg?)
<details>
<summary></summary>

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
A -> User: Resp: None
Deactivate A

@enduml

</details> -->
