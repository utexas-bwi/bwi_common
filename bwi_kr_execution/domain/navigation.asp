#include <iclingo>.
#program cumulative(n).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
facing(D,n) :- approach(D,n).
beside(D,n) :- approach(D,n).
at(R2,n) :- approach(D,n), at(R1,n-1), hasdoor(R2,D), acc(R1,R2).
:- approach(D,n), facing(D,n-1).
:- approach(D,n), at(R1,n-1), dooracc(R3,D,R2), not acc(R1,R3), not acc(R1,R2).

at(R2,n) :- gothrough(D,n), at(R1,n-1), dooracc(R1,D,R2).
% to deal with the elevator door - multiple elevator rooms sharing one door
-facing(D1,n) :- gothrough(D,n), door(D1).
:- gothrough(D,n), not facing(D,n-1).
:- gothrough(D,n), not open(D,n-1).
:- gothrough(D,n), at(R,n-1), not hasdoor(R,D).

open(D,n) :- opendoor(D,n).
:- opendoor(D,n), not facing(D,n-1).
:- opendoor(D,n), open(D,n-1).

facing(O,n) :- goto(O,n).
beside(O,n) :- goto(O,n).
at(R,n) :- goto(O,n), inside(O,R).
:- goto(O,n), inside(O,R1), at(R2,n-1), not acc(R1,R2).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%you can't be at two places at the some time
-at(L2,n):- at(L1,n), room(L2), L1 != L2.
%you can be facing only one door at a time
-facing(D2,n):- facing(D1,n), door(D2), D1 != D2.
%you can only be beside a door at any given time (if you delete this,
%the observations must also return -beside which doesn't happen at the moment.
-beside(D2,n):- beside(D1,n), door(D2), D1 != D2.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%at is inertial
at(L,n) :- at(L,n-1), not -at(L,n).
%facing is inertial
facing(D,n) :- facing(D,n-1), not -facing(D,n).
-facing(D,n) :- -facing(D,n-1), not facing(D,n).
% open is inertial
open(D,n) :- open(D,n-1), not -open(D,n).
-open(D,n) :- -open(D,n-1), not open(D,n).
% beside is inertial
beside(D,n) :- beside(D,n-1), not -beside(D,n).
-beside(D,n) :- -beside(D,n-1), not beside(D,n).

#show at/2.
#show open/2.
#show -open/2.
#show facing/2.
#show beside/2.

#show approach/2.
#show gothrough/2.
#show opendoor/2.
#show goto/2.
