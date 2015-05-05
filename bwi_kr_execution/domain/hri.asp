#cumulative n.

%action searchroom(P,R,I)  ask if person P is in room R
inroom(P,R,I+1) :- searchroom(P,R,I), person(P), room(R), I=0..n-2.

%:- searchroom(P,R,I), not facing(D,I) : hasdoor(R,D), not at(R,I), person(P), room(R), I=0..n-1.
%:- searchroom(P,R,I), facing(D,I), open(D,I), hasdoor(R,D), person(P), room(R), door(D), I=0..n-1.
%the rules above allow the robot execute the action outside of the room, in front of the door.
%I am making the robot enter the room, but switch back to the rules above to return to the previous behavior

%-searchroom(P,R,I) :- person(P), room(R), not searchroom(P,R,I), I=0..n-1.
%-askperson(P1,P2,I) :- person(P1), person(P2), not askperson(P1,P2,I), I=0..n-1.

:- searchroom(P,R,I), not at(R,I), I=0..n-1.

:- searchroom(P,R,I), inroom(P,R,I), person(P), room(R), I=0..n-1.
:- searchroom(P,R,I), -inroom(P,R,I), person(P), room(R), I=0..n-1.
:- searchroom(P,R,I), not canbeinroom(P,R), person(P), room(R), I=0..n-1.

%workaround to the problem of having the robot serach rooms and asking people when not necessary
%add findPersonTask(n-1) to the goal formula to allow these actions
:- searchroom(P,R,I), not findPersonTask(n-1), person(P), room(R), I=0..n-1.
:- askperson(P1,P2,I), not findPersonTask(n-1), person(P1), person(P2), I=0..n-1.
findPersonTask(I) :- findPersonTask(I).

%inroom is inertial
inroom(P,R,I+1) :- inroom(P,R,I), not -inroom(P,R,I+1), I=0..n-2.
-inroom(P,R,I+1) :- -inroom(P,R,I), not inroom(P,R,I+1), I=0..n-2.

%everyone can only be in one room at any given time
:- inroom(P,R1,I), inroom(P,R2,I), R1 != R2, I=0..n-1.

%fluent inoffice(P,I)
inoffice(P,I) :- inroom(P,R,I), hasoffice(P,R), person(P), room(R), I=0..n-1.
-inoffice(P,I) :- -inroom(P,R,I), hasoffice(P,R), person(P), room(R), I=0..n-1.

%fluent ingdc(P,I)
ingdc(P,I) :- inroom(P,R,I), person(P), room(R), I=0..n-1.
-ingdc(P,I) :- { not -inroom(P,R,I) : canbeinroom(P,R) }0, { not -know(P1,P,I) : canknow(P1,P) }0, person(P), I=0..n-1.

%action askperson(P1,P2,I)  ask P1 where P2 is
inroom(P2,R,I+1) :- askperson(P1,P2,I), at(R,I), person(P1), person(P2), room(R), not -inroom(P2,R,I+1), I=0..n-2.
:- askperson(P1,P2,I), not inroom(P1,R,I) : room(R), at(R,I), person(P1), person(P2), I=0..n-1.
:- askperson(P1,P2,I), inroom(P2,R,I), person(P1), person(P2), room(R), I=0..n-1.
:- askperson(P1,P2,I), not canknow(P1,P2), person(P1), person(P2), I=0..n-1.
:- askperson(P1,P2,I), -know(P1,P2,I), person(P1), person(P2), I=0..n-1.
:- inroom(P,R,I), not room(R), I=0..n-1.

%fluent know(P1,P2)  P1 knows where P2 is
know(P1,P2,I+1) :- askperson(P1,P2,I), person(P1), person(P2), I=0..n-2.
-know(P1,P2,I) :- -ingdc(P1,I), canknow(P1,P2), person(P1), person(P2), I=0..n-1.

%know is inertial
know(P1,P2,I+1) :- know(P1,P2,I), not -know(P1,P2,I+1), I=0..n-2.
-know(P1,P2,I+1) :- -know(P1,P2,I), not know(P1,P2,I+1), I=0..n-2.

%fluent inmeeting(P,M,I) person P is in meeting M
inmeeting(P,M,I) :- inroom(P,R,I), meeting(M,G,R), ingroup(P,G), person(P), group(G), room(R), I=0..n-1.

%action remind(P,M,R,I)
inmeeting(P,M,I+1) :- remind(P,M,R,I), meeting(M,G,R), ingroup(P,G), person(P), group(G), room(R), I=0..n-2.
:- remind(P,M,R1,I), not inroom(P,R2,I) : room(R2), at(R2,I), person(P), I=0..n-1.

%inmeeting is inertial
inmeeting(P,M,I+1) :- inmeeting(P,M,I), not -inmeeting(P,M,I+1), I=0..n-2.
-inmeeting(P,M,I+1) :- -inmeeting(P,M,I), not inmeeting(P,M,I+1), I=0..n-2.

%fluent inmeetingornowhere(P,M,I) person P is in meeting or not in gdc
inmeetingornowhere(P,M,I) :- inmeeting(P,M,I), meeting(M,G,R), I=0..n-1.
inmeetingornowhere(P,M,I) :- -ingdc(P,I), meeting(M,G,R), I=0..n-1.

%fluent allinmeeting(M,I)
%allinmeeting(M,I) :- { not inmeeting(P,M,I) : ingroup(P,G) }0, meeting(M,G,R), group(G), room(R), I=0..n-1.
allinmeeting(M,I) :- { not inmeetingornowhere(P,M,I) : ingroup(P,G) }0, at(R,I), meeting(M,G,R), group(G), room(R), I=0..n-1.
