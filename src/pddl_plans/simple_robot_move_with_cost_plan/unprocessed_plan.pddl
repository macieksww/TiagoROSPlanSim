Domain parsed
Problem parsed
Grounding..
Light Validation Completed
Delta time validation model:1
Delta time execution model:1.0
Delta time heuristic model:1.0
Delta Max:1
Simplification..
Grounding and Simplification finished
|A|:16
|P|:16
|E|:16
Running WA-STAR
Reachable actions and processes: |A U P U E|:48
Take all reachable actions
Actions used at init:16
h(n = s_0)=12.0
f(n) = 12.0 (Expanded Nodes: 0, Evaluated States: 0, Time: 0.048)
f(n) = 15.0 (Expanded Nodes: 1, Evaluated States: 4, Time: 0.055)
f(n) = 16.0 (Expanded Nodes: 3, Evaluated States: 9, Time: 0.063)
f(n) = 17.0 (Expanded Nodes: 7, Evaluated States: 19, Time: 0.074)
f(n) = 18.0 (Expanded Nodes: 15, Evaluated States: 29, Time: 0.083)
f(n) = 19.0 (Expanded Nodes: 31, Evaluated States: 59, Time: 0.101)
Problem Solved
Epsilon set to be:0.0
(0.00000) start-move Parameters: rico  - robot  ricochargerstation  - location  kitchen  - location  
(0.00000) end-move Parameters: rico  - robot  ricochargerstation  - location  kitchen  - location  
(0.00000,1.00000)------>waiting
(1.00000) start-move Parameters: rico  - robot  kitchen  - location  bathroom  - location  
(1.00000) end-move Parameters: rico  - robot  kitchen  - location  bathroom  - location  
(1.00000,2.00000)------>waiting
(2.00000) start-move Parameters: rico  - robot  bathroom  - location  livingroom  - location  
(2.00000) end-move Parameters: rico  - robot  bathroom  - location  livingroom  - location  
(2.00000,3.00000)------>waiting
(3.00000) start-move Parameters: rico  - robot  livingroom  - location  ricochargerstation  - location  
(3.00000) end-move Parameters: rico  - robot  livingroom  - location  ricochargerstation  - location  
(3.00000,4.00000)------>waiting

Resolution for validation:1.0
(Pddl+ semantics) Plan is valid:true
Plan-Length:12
Planning Time:376
Heuristic Time:51
Search Time:106
Expanded Nodes:37
States Evaluated:70
Duration:4.0000000
Total Cost:0.0
Fixed constraint violations during search (zero-crossing):0
Number of Dead-Ends detected:3
Number of Duplicates detected:23
Number of LP invocations:0
