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

caninside(P,R) :- hasoffice(P,R), person(P), room(R).

object(O) :- object(O,0).
inside(O,R) :- inside(O,R,0).
person(P) :- person(P,0).
caninside(P,R) :- caninside(P,R,0).
canbeside(P,O) :- canbeside(P,O,0).
caninside(P,R) :- canbeside(P,O), inside(O,R).

canlookfor(P) :- caninside(P,R).

%get rid of warning
person(P,0) :- person(P,0).
caninside(P,R,0) :- caninside(P,R,0).
canbeside(P,O,0) :- canbeside(P,O,0).
