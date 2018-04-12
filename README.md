# BosRepo

![Alt text](https://github.com/BosMathias/BosRepo/blob/master/README.md)
<details>
<summary></summary>

@startuml;
actor User;
participant "AANPASSING" as A;
participant "Second Class" as B;
participant "Last Class" as C;
User -> A: DoWork;
activate A;
A -> B: Create Request;
activate B;
B -> C: DoWork;
activate C;
C -> B: WorkDone;
destroy C;
B -> A: Request Created;
deactivate B;
A -> User: Done;
deactivate A;
@enduml

</details>
