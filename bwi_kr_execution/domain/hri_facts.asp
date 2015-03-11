
person(peter). 
person(ray). 
person(dana). 
person(kazunori). 
person(matteo). 
person(shiqi). 
person(jivko). 
person(stacy).
person(yuqian).

group(bwi).

ingroup(peter,bwi).
ingroup(matteo,bwi).
ingroup(shiqi,bwi).
ingroup(jivko,bwi).
ingroup(yuqian,bwi).

meeting(bwi_meeting,bwi,l3_414b).

hasoffice(peter,l3_508). 
hasoffice(ray,l3_512).
hasoffice(dana,l3_510). 
hasoffice(kazunori,l3_402). 
hasoffice(matteo,l3_418).
hasoffice(shiqi,l3_420).
hasoffice(jivko,l3_432). 
hasoffice(stacy,l3_502). 

canbeinroom(P,R) :- hasoffice(P,R), person(P), room(R).
canbeinroom(P,l3_414b) :- ingroup(P,bwi).
%canknow(P1,P2) :- ingroup(P1,G), ingroup(P2,G), group(G).

canknow(P1,P2) :- canknow(P1,P2).

#hide person/1.
#hide hasoffice/2.
#hide canbeinroom/2.
#hide group/1.
#hide ingroup/2.
#hide meeting/3.
