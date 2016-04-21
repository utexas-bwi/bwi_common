#program base.
person(peter). 
person(ray). 
person(dana). 
person(kazunori). 
person(matteo). 
person(shiqi). 
person(jivko). 
person(stacy).
person(yuqian).
person(max).
person(pato).

group(bwi).

ingroup(peter,bwi).
ingroup(matteo,bwi).
ingroup(shiqi,bwi).
ingroup(jivko,bwi).
ingroup(yuqian,bwi).
ingroup(pato,bwi).
ingroup(max,bwi).


hasoffice(peter,l3_508). 
hasoffice(ray,l3_512).
hasoffice(dana,l3_510). 
hasoffice(kazunori,l3_402). 
hasoffice(matteo,l3_418).
hasoffice(shiqi,l3_420).
hasoffice(jivko,l3_432). 
hasoffice(stacy,l3_502).

%students in the lab
hasoffice(yuqian,l3_414b).
hasoffice(pato,l3_414b).
hasoffice(max,l3_414b).

canbeinroom(P,R) :- hasoffice(P,R), person(P), room(R).
canbeinroom(P,l3_414b) :- ingroup(P,bwi).

canknow(P1,P2) :- ingroup(P1,G), ingroup(P2,G), P1 != P2, group(G).

canknow(P2,P1) :- canknow(P1,P2).

meeting(M,G,R) :- meeting(M,G,R). %here for when not using meetings

%#show person/1.
%#show hasoffice/2.
%#show canbeinroom/2.
%#show canknow/2.
%#show group/1.
%#show ingroup/2.
%#show meeting/3.
