#program cumulative(n).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

at(R,n) :- changefloor(R,n).
beside(D,n) :- changefloor(R,n), hasdoor(R,D).
facing(D,n) :- changefloor(R,n), hasdoor(R,D).
open(D,n) :- changefloor(R,n), hasdoor(R,D).

:- changefloor(R,n), at(R,n-1).
:- changefloor(R1,n), at(R2,n-1), not sameroom(R1,R2).
:- changefloor(R,n), room(R), not elevroom(R).
:- changefloor(R1,n), at(R,n-1), hasdoor(R,D), not facing(D,n-1), elevroom(R), door(D). 
:- changefloor(R1,n), floor(R1,F1), at(R2,n-1), floor(R2,F2), upward(n-1), F1<F2. 
:- changefloor(R1,n), floor(R1,F1), at(R2,n-1), floor(R2,F2), -upward(n-1), F1>F2. 

open(D,n) :- callelevator(E,O,n), elevator(E), elevhasdoor(E,D), orientation(O), beside(D,n-1). 

%you can't call the elevator if the door you are beside is not an elevator door
:- callelevator(E,O,n),  beside(D,n-1), not elevhasdoor(E,D), orientation(O). 
%you have to be beside at least a door
:- callelevator(E,O,n),  0{beside(D,n-1) : door(D)}0. 
:- callelevator(E,O,n), at(R,n-1), elevroom(R). 

upward(n) :- callelevator(E,up,n). 
-upward(n) :- callelevator(E,down,n). 


%you can't open an elevator door with opendoor...
:- opendoor(D,n), elevdoor(D).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Default dynamics
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

upward(n) :- upward(n-1), not -upward(n), at(R,n), elevroom(R). 
-upward(n) :- -upward(n-1), not upward(n), at(R,n), elevroom(R). 


%this dynamics for the elevetor doors looked interesting and could even work,
%but since doors open and close by themselves the robot must have an action that
% doesn't change anything and just makes it wait for the doors to open, which we currently don't have.
%Instead, changefloor now also opens the door.

%doors open by themselves when you are in the elevetor!
%open(D,n) :- elevdoor(D), at(R,n), elevroom(R), hasdoor(R,D), not -open(D,n).

%doors close by themselves when you leave the elevetor
-open(D,n) :- elevdoor(D), at(R2,n), at(R1,n-1), elevroom(R1), not elevroom(R2).

%hide fluents implied by others
#show upward/1.
#show -upward/1.
