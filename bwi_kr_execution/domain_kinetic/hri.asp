#program step(n).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Actions
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%action knock(D,I)  ask if the robot can come into an office when the door is open
accessgranted(D,I) :- knock(D,I), I>0, I=n-1.
:- knock(D,I), not facing(D,I-1).
:- knock(D,I), not open(D,I-1).
%:- knock(D,I), 0{hasdoor(R,D):hasoffice(P,R)}0.
:- knock(D,I), -accessgranted(D,I-1).

%action searchperson(P,R,O,I)  ask if person P is in room R
inside(P,R,I) :- searchperson(P,R,O,I), I>0, I=n-1.
near(P,O,I) :- searchperson(P,R,O,I), I>0, I=n-1.
:- searchperson(P,R,O,I), not at(R,I-1), I>0, I=n-1.
:- searchperson(P,R,O,I), -inside(P,R,I-1), I>0, I=n-1.
:- searchperson(P,R,O,I), not caninside(P,R), person(P), room(R), I>0, I=n-1.
:- searchperson(P,R,O,I), not canbeside(P,O), person(P), object(O), I>0, I=n-1.
:- searchperson(P,R,O,I), not beside(O,I-1), I>0, I=n-1.
%can only bother bwi people
%:- searchperson(P,R,O,I), not ingroup(P,bwi), hasoffice(P,R), person(P), I>0, I=n-1.

%action delivermessage(P,M,I) deliver message M to person P
messagedelivered(P,M,I) :- delivermessage(P,M,I), I>0, I=n-1.
:- delivermessage(P,M,I), at(R,I-1), not inside(P,R,I-1).
:- delivermessage(P,M,I), near(P,O,I-1), not beside(O,I-1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%person is busy if in office but denied access
busy(P,I) :- inside(P,R,I), -accessgranted(D,I), hasdoor(R,D), hasoffice(P,R).

%-found in a room if all doors are closed
-alldoorsclosed(R,I) :- not -open(D,I), not badDoor(D), not -accessgranted(D,I), hasdoor(R,D), I>0, I=n-1.
-found(P,R,I) :- not -alldoorsclosed(R,I), hasdoor(R,D1), not busy(P,I), not at(R,I), caninside(P,R), I>0, I=n-1.
-found(P,R,I) :- -inside(P,R,I), I>0, I=n-1.
-found(P,O,I) :- -near(P,O,I), I>0, I=n-1.
%found if inside is true for a room
found(P,I) :- inside(P,R,I), room(R), I>0, I=n-1.
%-found if -found in all possible rooms
-finished(P,I) :- not -found(P,R,I), caninside(P,R), person(P), I>0, I=n-1.
-found(P,I) :- not -finished(P,I), person(P), I>0, I=n-1.

%closed office door
closedofficedoor(D,I) :- -open(D,I), hasdoor(R,D), hasoffice(P,R), not at(R,I-1), I>=0, I=n-1.

lookingfor(P,I) :- lookingfor(P,I).
-message(P,M,I) :- -message(P,M,I).

%reset -open when we want to look for a person again
-knowclosed(D,1) :- lookingfor(P,1), caninside(P,R), hasdoor(R,D), not facing(D,0), not badDoor(D).

:- lookingfor(P,1), not canlookfor(P).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%inside is inertial
inside(O,R,I) :- inside(O,R,I-1), not -inside(O,R,I), I>0, I=n-1.
-inside(O,R,I) :- -inside(O,R,I-1), not inside(O,R,I), not lookingfor(O,1), I>0, I=n-1.
%you can't be at two places at the some time
-inside(O,R2,I):- inside(O,R1,I), inside(O,R2,I-1), R1 != R2, I>0, I=n-1.

%accessgranted is inertial
%accessgranted(D,I) :- accessgranted(D,I-1), not -accessgranted(D,I), I>0, I=n-1.
-accessgranted(D,I) :- -accessgranted(D,I-1), not accessgranted(D,I), not -knowclosed(D,I), I>0, I=n-1.

%message is inertial; maybe update -message after successful deliver
message(P,M,I) :- message(P,M,I-1), not -message(P,M,I), I>0, I=n-1.

%near is inertial
near(O1,O2,I) :- near(O1,O2,I-1), not -near(O1,O2,I), I>0, I=n-1.
-near(O1,O2,I) :- -near(O1,O2,I-1), not near(O1,O2,I), I>0, I=n-1.

%caninside is inertial
caninside(P,R,I) :- caninside(P,R,I-1), I>0, I=n-1.

%canbeside is inertial
canbeside(P,R,I) :- canbeside(P,R,I-1), I>0, I=n-1.

%person is inertial
person(P,I) :- person(P,I-1), I>0, I=n-1.

%object is inertial
object(O,I) :- object(O,I-1), I>0, I=n-1.

%messagedelivered is inertial
messagedelivered(P,M,I) :- messagedelivered(P,M,I-1), not -messagedelivered(P,M,I), I>0, I=n-1.
-messagedelivered(P,M,I) :- -messagedelivered(P,M,I-1), not messagedelivered(P,M,I), I>0, I=n-1.

#show inside/3.
#show -inside/3.
#show found/2.
#show -found/2.
#show accessgranted/2.
#show -accessgranted/2.
#show message/3.
#show messagedelivered/3.
#show near/3.
#show caninside/3.
#show canbeside/3.
#show busy/2.
#show object/2.
#show person/2.
