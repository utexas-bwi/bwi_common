#cumulative n.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


facing(D,I+1) :- approach(D,I), door(D), I=0..n-2.
beside(D,I+1) :- approach(D,I), door(D), I=0..n-2.
at(R2,I+1) :- approach(D,I), at(R1,I), hasdoor(R2,D), acc(R1,R2), I=0..n-2.
:- approach(D,I), facing(D,I), door(D), I=0..n-1.
:- approach(D,I), door(D), at(R1,I), dooracc(R3,D,R2), not acc(R1,R3), 
   not acc(R1,R2), I=0..n-1.


at(R2,I+1) :- gothrough(D,I),  at(R1,I), dooracc(R1,D,R2), I=0..n-2.
% to deal with the elevator door - multiple elevator rooms sharing one door
-facing(D1,I+1) :- gothrough(D,I), door(D1), I=0..n-2.
:- gothrough(D,I), not facing(D,I), door(D), I=0..n-1.
:- gothrough(D,I), not open(D,I), door(D), I=0..n-1.
:- gothrough(D,I), at(R,I), not hasdoor(R,D), door(D), room(R), I=0..n-1.


open(D,I+1) :- opendoor(D,I), door(D), I=0..n-2.
:- opendoor(D,I), not facing(D,I), door(D), I=0..n-1.
:- opendoor(D,I), open(D,I), door(D), I=0..n-1.


facing(O,I+1) :- goto(O,I), object(O), I=0..n-2.                                
beside(O,I+1) :- goto(O,I), object(O), I=0..n-2.                                
at(R,I+1) :- goto(O,I), object(O), inside(O,R), room(R), I=0..n-2.              
:- goto(O,I), inside(O,R1), at(R2,I), not acc(R1,R2), object(O), room(R1), room(R2), I=0..n-1.



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%you can't be at two places at the some time
-at(L2,I):- at(L1,I), room(L1), room(L2), L1 != L2, I=0..n-1.


%you can be facing only one door at a time
-facing(D2,I):- facing(D1,I), door(D2), D1 != D2, I=0..n-1.

%you can only be beside a door at any given time (if you delete this, 
%the observations must also return -beside which doesn't happen at the moment.
-beside(D2,I):- beside(D1,I), door(D1), door(D2), D1 != D2, I=0..n-1.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%  Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%at is inertial
at(L,I+1) :- at(L,I), not -at(L,I+1), room(L), I=0..n-2.

%facing is inertial
facing(D,I+1) :- facing(D,I), not -facing(D,I+1), I=0..n-2.
-facing(D,I+1) :- -facing(D,I), not facing(D,I+1), I=0..n-2.

% open is inertial
open(D,I+1) :- open(D,I), not -open(D,I+1), I=0..n-2.
-open(D,I+1) :- -open(D,I), not open(D,I+1), I=0..n-2.

% beside is inertial
beside(D,I+1) :- beside(D,I), not -beside(D,I+1), I=0..n-2.
-beside(D,I+1) :- -beside(D,I), not beside(D,I+1), I=0..n-2.

%hide fluents implied by others
#hide -at/2.
#hide -facing/2.
#hide -beside/2.
