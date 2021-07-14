#program cumulative(n).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

at(R,I) :- changefloor(R,I), I>0, I=n-1.
beside(D,I) :- changefloor(R,I), hasdoor(R,D), I>0, I=n-1.
facing(D,I) :- changefloor(R,I), hasdoor(R,D), I>0, I=n-1.
open(D,I) :- changefloor(R,I), hasdoor(R,D), I>0, I=n-1.

:- changefloor(R,I), at(R,I-1).
:- changefloor(R1,I), at(R2,I-1), not sameroom(R1,R2).
:- changefloor(R,I), room(R), not elevroom(R).
:- changefloor(R1,I), at(R,I-1), hasdoor(R,D), not facing(D,I-1), elevroom(R), door(D). 
:- changefloor(R1,I), floor(R1,F1), at(R2,I-1), floor(R2,F2), upward(I-1), F1<F2. 
:- changefloor(R1,I), floor(R1,F1), at(R2,I-1), floor(R2,F2), -upward(I-1), F1>F2. 

open(D,I) :- callelevator(E,O,I), elevator(E), elevhasdoor(E,D), orientation(O), beside(D,I-1), I>0, I=n-1. 

%you can't call the elevator if the door you are beside is not an elevator door
:- callelevator(E,O,I),  beside(D,I-1), not elevhasdoor(E,D), orientation(O), I>0, I=n-1. 
%you have to be beside at least a door
:- callelevator(E,O,I),  0{beside(D,I-1) : door(D)}0. 
:- callelevator(E,O,I), at(R,I-1), elevroom(R). 

upward(I) :- callelevator(E,up,I), I>0, I=n-1. 
-upward(I) :- callelevator(E,down,I), I>0, I=n-1. 


%you can't open an elevator door with opendoor...
:- opendoor(D,I), elevdoor(D).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Default dynamics
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

upward(I) :- upward(I-1), not -upward(I), at(R,I), elevroom(R), I>0, I=n-1. 
-upward(I) :- -upward(I-1), not upward(I), at(R,I), elevroom(R), I>0, I=n-1. 


%this dynamics for the elevetor doors looked interesting and could even work,
%but since doors open and close by themselves the robot must have an action that
% doesn't change anything and just makes it wait for the doors to open, which we currently don't have.
%Instead, changefloor now also opens the door.

%doors open by themselves when you are in the elevetor!
%open(D,I) :- elevdoor(D), at(R,I), elevroom(R), hasdoor(R,D), not -open(D,I), I>0, I=n-1.

%doors close by themselves when you leave the elevetor
-open(D,I) :- elevdoor(D), at(R2,I), at(R1,I-1), elevroom(R1), not elevroom(R2), I>0, I=n-1.

%hide fluents implied by others
#show upward/1.
#show -upward/1.
