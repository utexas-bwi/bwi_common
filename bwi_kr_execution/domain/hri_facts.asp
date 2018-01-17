#program base.
person(peter). 
person(ray). 
person(dana). 
person(justin). 
person(garrett).
person(stacy).

hasoffice(peter,l3_508). 
hasoffice(ray,l3_512).
hasoffice(dana,l3_510). 
hasoffice(justin,l3_402). 
hasoffice(garrett,l3_422).
hasoffice(stacy,l3_502).

object(O) :- locationmarker(P,O,0).
inside(O,R) :- locationmarker(P,O,0), inroom(P,R,0).
person(P) :- possiblelocation(P,R,0).
possiblelocation(P,R) :- possiblelocation(P,R,0), room(R).

possiblelocation(P,R) :- hasoffice(P,R), person(P), room(R).

possiblelocation(P) :- possiblelocation(P,R).
%:- possiblelocation(P,R,0), not room(R).


