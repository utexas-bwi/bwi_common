#cumulative n.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


at(R,I+1) :- changefloor(R,I), I=0..n-2.
beside(D,I+1) :- changefloor(R,I), hasdoor(R,D),I=0..n-2.
facing(D,I+1) :- changefloor(R,I), hasdoor(R,D),I=0..n-2.
open(D,I+1) :- changefloor(R,I), hasdoor(R,D),I=0..n-2.

:- changefloor(R,I), at(R,I), I=0..n-1.
:- changefloor(R1,I), at(R2,I), not sameroom(R1,R2), I=0..n-1.
:- changefloor(R,I), room(R), not elevroom(R), I=0..n-1.
:- changefloor(R1,I), at(R,I), hasdoor(R,D), not facing(D,I), elevroom(R), door(D), I=0..n-1. 
:- changefloor(R1,I), floor(R1, F1), at(R2, I), floor(R2, F2), upward(I), F1<F2, I=0..n-1. 
:- changefloor(R1,I), floor(R1, F1), at(R2, I), floor(R2, F2), -upward(I), F1>F2, I=0..n-1. 

open(D,I+1) :- callelevator(E,O,I), elevator(E), elevhasdoor(E,D), orientation(O), beside(D,I), I=0..n-2. 

%you can't call the elevator if the door you are beside is not an elevator door
:- callelevator(E,O,I),  beside(D,I), not elevhasdoor(E,D), orientation(O), I=0..n-1. 
%you have to be beside at least a door
:- callelevator(E,O,I),  0{beside(D,I) : door(D)}0, I=0..n-1. 
:- callelevator(E,O,I), at(R,I), elevroom(R), I=0..n-1. 

upward(I+1) :- callelevator(E,up,I), I=0..n-2. 
-upward(I+1) :- callelevator(E,down,I), I=0..n-2. 


%you can't open an elevator door with opendoor...
:- opendoor(D,I), elevdoor(D), I=0..n-1.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Default dynamics
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

upward(I+1) :- upward(I), not -upward(I+1), at(R,I+1), elevroom(R),  I=0..n-2. 
-upward(I+1) :- -upward(I), not upward(I+1), at(R,I+1), elevroom(R), I=0..n-2. 


%this dynamics for the elevetor doors looked interesting and could even work,
%but since doors open and close by themselves the robot must have an action that
% doesn't change anything and just makes it wait for the doors to open, which we currently don't have.
%Instead, changefloor now also opens the door.

%doors open by themselves when you are in the elevetor!
%open(D,I+1) :- elevdoor(D), at(R,I+1), elevroom(R), hasdoor(R,D), not -open(D,I+1), I=0..n-2.

%doors close by themselves when you leave the elevetor
-open(D,I+1) :- elevdoor(D), at(R2,I+1), at(R1,I), elevroom(R1), not elevroom(R2),  I=0..n-2.

%hide fluents implied by others

