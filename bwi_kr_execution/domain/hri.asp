#program cumulative(n).

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

%action searchroom(P,R,I)  ask if person P is in room R
inroom(P,R,I) :- searchroom(P,R,I), I>0, I=n-1.
:- searchroom(P,R,I), not at(R,I-1), I>0, I=n-1.
:- searchroom(P,R,I), -inroom(P,R,I-1), I>0, I=n-1.
:- searchroom(P,R,I), not possiblelocation(P,R), person(P), room(R), I>0, I=n-1.
%can only bother bwi people
%:- searchroom(P,R,I), not ingroup(P,bwi), hasoffice(P,R), person(P), I>0, I=n-1.

%action delivermessage(P,M,I) deliver message M to person P
messagedelivered(P,M,I) :- delivermessage(P,M,I), I>0, I=n-1.
:- delivermessage(P,M,I), at(R,I-1), not inroom(P,R,I-1).
:- delivermessage(P,M,I), locationmarker(P,O,I-1), not beside(O,I-1).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Static laws
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%person is busy if in office but denied access
busy(P,I) :- inroom(P,R,I), -accessgranted(D,I), hasdoor(R,D), hasoffice(P,R).

%-found in a room if all doors are closed
-alldoorsclosed(R,I) :- not -open(D,I), not badDoor(D), not -accessgranted(D,I), hasdoor(R,D), I>0, I=n-1.
-found(P,R,I) :- not -alldoorsclosed(R,I), hasdoor(R,D1), not busy(P,I), not at(R,I), possiblelocation(P,R), I>0, I=n-1.
-found(P,R,I) :- -inroom(P,R,I), I>0, I=n-1.
%found if inroom is true for a room
found(P,I) :- inroom(P,R,I), room(R), I>0, I=n-1.
%-found if -found in all possible rooms
-finished(P,I) :- not -found(P,R,I), possiblelocation(P,R), person(P), I>0, I=n-1.
-found(P,I) :- not -finished(P,I), person(P), I>0, I=n-1.

%closed office door
closedofficedoor(D,I) :- -open(D,I), hasdoor(R,D), hasoffice(P,R), not at(R,I-1), I>=0, I=n-1.

%reset -open when we want to look for a person again
-knowclosed(D,1) :- lookingfor(P,1), possiblelocation(P,R), hasdoor(R,D), not facing(D,0), not badDoor(D).

:- lookingfor(P,1), not possiblelocation(P).

lookingfor(P,I) :- lookingfor(P,I).
-message(P,M,I) :- -message(P,M,I).
-locationmarker(P,O,I) :- -locationmarker(P,O,I).
-possiblelocation(P,R,I) :- -possiblelocation(P,R,I).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Inertia
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%inroom is inertial
inroom(P,R,I) :- inroom(P,R,I-1), not -inroom(P,R,I), I>0, I=n-1.
-inroom(P,R,I) :- -inroom(P,R,I-1), not inroom(P,R,I), not lookingfor(P,1), I>0, I=n-1.
%you can't be at two places at the some time
%-inroom(P,R2,I):- inroom(P,R1,I), inroom(P,R2,I-1), R1 != R2, I>0, I=n-1.

%accessgranted is inertial
%accessgranted(D,I) :- accessgranted(D,I-1), not -accessgranted(D,I), I>0, I=n-1.
-accessgranted(D,I) :- -accessgranted(D,I-1), not accessgranted(D,I), not -knowclosed(D,I), I>0, I=n-1.

%message is inertial; maybe update -message after successful deliver
message(P,M,I) :- message(P,M,I-1), not -message(P,M,I), I>0, I=n-1.

%location marker is inertial
locationmarker(P,O,I) :- locationmarker(P,O,I-1), not -locationmarker(P,O,I), I>0, I=n-1.

%possible location is inertial
possiblelocation(P,R,I) :- possiblelocation(P,R,I-1), not -possiblelocation(P,R,I), I>0, I=n-1.

%messagedelivered is inertial
messagedelivered(P,M,I) :- messagedelivered(P,M,I-1), not -messagedelivered(P,M,I), I>0, I=n-1.
-messagedelivered(P,M,I) :- -messagedelivered(P,M,I-1), not messagedelivered(P,M,I), I>0, I=n-1.

#show inroom/3.
#show -inroom/3.
#show found/2.
#show -found/2.
#show accessgranted/2.
#show -accessgranted/2.
#show message/3.
#show messagedelivered/3.
#show locationmarker/3.
#show possiblelocation/3.
#show busy/2.
