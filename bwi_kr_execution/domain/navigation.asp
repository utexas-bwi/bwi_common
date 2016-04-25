#include <iclingo>.

badDoor(d3_418).
badDoor(d3_414b1).
badDoor(d3_414b2).


#program cumulative(n).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
facing(D,I) :- approach(D,I), I>0, I=n-1.
beside(D,I) :- approach(D,I), I>0, I=n-1.
at(R2,I) :- approach(D,I), at(R1,I-1), hasdoor(R2,D), acc(R1,R2), I>0, I=n-1.
:- approach(D,I), facing(D,I-1).
:- approach(D,I), at(R1,I-1), dooracc(R3,D,R2), not acc(R1,R3), not acc(R1,R2).
besidedoor(I) :- beside(D,I), door(D), I>=0, I=n-1.
knowbeside(I) :- besidedoor(I), I>=0, I=n-1.
beside(nothing,I) :- not besidedoor(I), not besidedoor(I-1), I>=0, I=n-1.
knowbeside(I) :- beside(nothing,I), I>=0, I=n-1.
%-beside(D) :- beside(nothing,I), door(D), I>=0, I=n-1.
:- approach(D,I), not knowbeside(I-1).
-beside(nothing,I) :- beside(D,I), door(D), I>=0, I=n-1.

at(R2,I) :- gothrough(D,I), at(R1,I-1), dooracc(R1,D,R2), I>0, I=n-1.
% to deal with the elevator door - multiple elevator rooms sharing one door
-facing(D1,I) :- gothrough(D,I), door(D1), I>0, I=n-1.
:- gothrough(D,I), not facing(D,I-1).
:- gothrough(D,I), not open(D,I-1).
:- gothrough(D,I), at(R,I-1), not hasdoor(R,D).


open(D,I) :- opendoor(D,I), I>0, not badDoor(D), I=n-1.
open(D,I) | -open(D,I):- opendoor(D,I), I>0, badDoor(D), I=n-1.
:- opendoor(D,I), not facing(D,I-1).
:- opendoor(D,I), open(D,I-1).

facing(O,I) :- goto(O,I), I>0, I=n-1.
beside(O,I) :- goto(O,I), I>0, I=n-1.
at(R,I) :- goto(O,I), inside(O,R), I>0, I=n-1.
:- goto(O,I), inside(O,R1), at(R2,I-1), not acc(R1,R2).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%you have to be somewhere
%:- 0{at(R,I):room(R)}0.
%you can't be at two places at the some time
-at(L2,I):- at(L1,I), room(L2), L1 != L2, I>0, I=n-1.
%you can be facing only one door at a time
-facing(D2,I):- facing(D1,I), door(D2), D1 != D2, I>0, I=n-1.
%you can only be beside a door at any given time (if you delete this,
%the observations must also return -beside which doesn't happen at the moment.
-beside(D2,I):- beside(D1,I), door(D2), D1 != D2, I>0, I=n-1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%at is inertial
at(L,I) :- at(L,I-1), not -at(L,I), I>0, I=n-1.
%facing is inertial
facing(D,I) :- facing(D,I-1), not -facing(D,I), I>0, I=n-1.
-facing(D,I) :- -facing(D,I-1), not facing(D,I), I>0, I=n-1.
% open is inertial
open(D,I) :- open(D,I-1), not -open(D,I), I>0, I=n-1.
-open(D,I) :- -open(D,I-1), not open(D,I), I>0, I=n-1.
% beside is inertial
beside(D,I) :- beside(D,I-1), not -beside(D,I), I>0, I=n-1.
-beside(D,I) :- -beside(D,I-1), not beside(D,I), I>0, I=n-1.

#show at/2.
#show open/2.
#show -open/2.
#show facing/2.
#show beside/2.
