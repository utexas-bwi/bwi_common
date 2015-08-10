#program cumulative(n).

%action searchroom(P,R,I)  ask if person P is in room R
inroom(P,R,n) :- searchroom(P,R,n), person(P), room(R).

%:- searchroom(P,R,n), not facing(D,n-1) : hasdoor(R,D), not at(R,n-1), person(P), room(R).
%:- searchroom(P,R,n), facing(D,n-1), open(D,n-1), hasdoor(R,D), person(P), room(R), door(D).
%the rules above allow the robot execute the action outside of the room, in front of the door.
%I am making the robot enter the room, but switch back to the rules above to return to the previous behavior

:- searchroom(P,R,n), not at(R,n-1).

:- searchroom(P,R,n), inroom(P,R,n-1), person(P), room(R).
:- searchroom(P,R,n), -inroom(P,R,n-1), person(P), room(R).
:- searchroom(P,R,n), not canbeinroom(P,R), person(P), room(R).

%workaround to the problem of having the robot search rooms and asking people when not necessary
%add findPersonTask(n-1) to the goal formula to allow these actions
%#volatile n.
#program find(n).
#external findPersonTask(n).
:- searchroom(P,R,n), not findPersonTask(n), person(P), room(R).
:- askperson(P1,P2,n), not findPersonTask(n), person(P1), person(P2).
findPersonTask(I) :- findPersonTask(I).

#program cumulative(n).

%inroom is inertial
inroom(P,R,n) :- inroom(P,R,n-1), not -inroom(P,R,n).
-inroom(P,R,n) :- -inroom(P,R,n-1), not inroom(P,R,n).

%everyone can only be in one room at any given time
:- inroom(P,R1,n), inroom(P,R2,n), R1 != R2.

%fluent inoffice(P,I)
inoffice(P,n) :- inroom(P,R,n), hasoffice(P,R), person(P), room(R).
-inoffice(P,n) :- -inroom(P,R,n), hasoffice(P,R), person(P), room(R).

%fluent ingdc(P,I)
ingdc(P,n) :- inroom(P,R,n), person(P), room(R).
-ingdc(P,n) :- { not -inroom(P,R,n) : canbeinroom(P,R) }0, { not -know(P1,P,n) : canknow(P1,P) }0, person(P).

%action askperson(P1,P2,I)  ask P1 where P2 is
inroom(P2,R,n) :- askperson(P1,P2,n), at(R,n-1), person(P1), person(P2), room(R), not -inroom(P2,R,n).
:- askperson(P1,P2,n), not inroom(P1,R,n-1) : room(R), at(R,n-1), person(P1), person(P2).
:- askperson(P1,P2,n), inroom(P2,R,n-1), person(P1), person(P2), room(R).
:- askperson(P1,P2,n), not canknow(P1,P2), person(P1), person(P2).
:- askperson(P1,P2,n), -know(P1,P2,n-1), person(P1), person(P2).
:- inroom(P,R,n), not room(R).

%fluent know(P1,P2)  P1 knows where P2 is
know(P1,P2,n) :- askperson(P1,P2,n), person(P1), person(P2).
-know(P1,P2,n) :- -ingdc(P1,n), canknow(P1,P2), person(P1), person(P2).

%know is inertial
know(P1,P2,n) :- know(P1,P2,n-1), not -know(P1,P2,n).
-know(P1,P2,n) :- -know(P1,P2,n-1), not know(P1,P2,n).

%fluent inmeeting(P,M,I) person P is in meeting M
inmeeting(P,M,n) :- inroom(P,R,n), meeting(M,G,R), ingroup(P,G), person(P), group(G), room(R).

%action remind(P,M,R,I)
inmeeting(P,M,n) :- remind(P,M,R,n), meeting(M,G,R), ingroup(P,G), person(P), group(G), room(R).
:- remind(P,M,R1,n), not inroom(P,R2,n-1) : room(R2), at(R2,n-1), person(P).

%inmeeting is inertial
inmeeting(P,M,n) :- inmeeting(P,M,n-1), not -inmeeting(P,M,n).
-inmeeting(P,M,n) :- -inmeeting(P,M,n-1), not inmeeting(P,M,n).

%fluent inmeetingornowhere(P,M,I) person P is in meeting or not in gdc
inmeetingornowhere(P,M,n) :- inmeeting(P,M,n), meeting(M,G,R).
inmeetingornowhere(P,M,n) :- -ingdc(P,n), meeting(M,G,R).

%fluent allinmeeting(M,I)
%allinmeeting(M,n) :- { not inmeeting(P,M,n) : ingroup(P,G) }0, meeting(M,G,R), group(G), room(R).
allinmeeting(M,n) :- { not inmeetingornowhere(P,M,n) : ingroup(P,G) }0, meeting(M,G,R), group(G), room(R).

#show inroom/3.
#show -inroom/3.
#show inoffice/2.
#show -inoffice/2.
#show ingdc/2.
#show -ingdc/2.
#show know/3.
#show -know/3.
#show inmeeting/3.
#show -inmeeting/3.
#show inmeetingornowhere/3.
#show allinmeeting/2.

#show searchroom/3.
#show askperson/3.
#show remind/4.
