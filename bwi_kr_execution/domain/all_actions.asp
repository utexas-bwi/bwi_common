#program cumulative(n).

1{
approach(D1,n) : door(D1);
gothrough(D2,n) : door(D2);
opendoor(D3,n) : door(D3);
goto(O,n) : object(O);
callelevator(E,U,n) : elevator(E) , orientation(U); 
changefloor(R,n) : room(R);
searchroom(P,R1,n): person(P) , room(R1);
askperson(P1,P2,n): person(P1) , person(P2);
remind(P3,M,R2,n) : person(P3) , meeting(M,G,R2) , room(R2)
}1 :- not noop(n).

noop(n) :- noop(n).
