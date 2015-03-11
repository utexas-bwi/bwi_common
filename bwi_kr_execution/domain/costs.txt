#cumulative n.

cost(@dis(X,Y,Z),I) :- approach(X,I), beside(Y,I), door(X), door(Y), at(Z,I).
cost(10,I) :- approach(X,I), {beside(Y,I)}0.
cost(1,I) :- gothrough(D,I). 
cost(1,I) :- opendoor(D,I).
cost(2,I) :- searchroom(P,R,I).
cost(2,I) :- askperson(P1,P2,I).

#minimize[cost(X,Y)=X@1].
