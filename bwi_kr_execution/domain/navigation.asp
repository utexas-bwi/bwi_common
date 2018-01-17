#include <iclingo>.
#program cumulative(n).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
facing(D,I) :- approach(D,I), I>0, I=n-1.
beside(D,I) :- approach(D,I), I>0, I=n-1.
at(R2,I) :- approach(D,I), at(R1,I-1), hasdoor(R2,D), acc(R1,R2), I>0, I=n-1.
:- approach(D,I), at(R1,I-1), dooracc(R3,D,R2), not acc(R1,R3), not acc(R1,R2).

at(R2,I) :- gothrough(D,I), at(R1,I-1), dooracc(R1,D,R2), I>0, I=n-1.
-facing(D,I) :- gothrough(D,I), facing(D,I-1), I>0, I=n-1.
:- gothrough(D,I), not facing(D,I-1).
:- gothrough(D,I), not open(D,I-1).
%cannot go through if the person in office is not available
%:- gothrough(D,I), at(R1,I-1), dooracc(R1,D,R2), hasoffice(P,R2), not accessgranted(D,I-1).
:- gothrough(D,I), at(R1,I-1), dooracc(R1,D,R2), hasoffice(P,R2), not accessgranted(D,I-1).

open(D,I) :- opendoor(D,I), I>0, I=n-1.
facing(D,I) :- opendoor(D,I), I>0, I=n-1.
beside(D,I) :- opendoor(D,I), I>0, I=n-1.
%at(R,I) :- opendoor(D,I), at(R,I-1), I>0, I=n-1.
:- opendoor(D,I), badDoor(D).
:- opendoor(D,I), not facing(D,I-1).
:- opendoor(D,I), closedofficedoor(D,I-1).
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
%you can't be at two places at the some time
-at(L2,I):- at(L1,I), room(L2), L1 != L2, I>0, I=n-1.

%you can only be beside a door at any given time (if you delete this,
%the observations must also return -beside which doesn't happen at the moment.
-beside(D,I) :- beside(D,I-1), at(R,I), not hasdoor(R,D), door(D), I>0, I=n-1.
-beside(O2,I):- beside(O1,I), beside(O2,I-1), O1 != O2, I>0, I=n-1.
-beside(O,I) :- beside(O,I-1), at(R,I), not inside(O,R), object(O), I>0, I=n-1.
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
%you can be facing only one door at a time
-facing(O2,I):- facing(O1,I), facing(O2,I-1), O1 != O2, I>0, I=n-1.
%-facing(D,I) :- facing(D,I-1), at(R,I), not hasdoor(R,D), I>0, I=n-1.
% open is inertial
open(D,I) :- open(D,I-1), not -open(D,I), I>0, I=n-1.
-open(D,I) :- -open(D,I-1), not open(D,I), not -knowclosed(D,I), I>0, I=n-1.
% beside is inertial
beside(D,I) :- beside(D,I-1), not -beside(D,I), I>0, I=n-1.
%-beside(D,I) :- -beside(D,I-1), not beside(D,I), I>0, I=n-1.

#show at/2.
#show open/2.
#show -open/2.
#show facing/2.
#show beside/2.
