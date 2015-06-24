#program step(n).

1{
approach(D1,n) : door(D1);
gothrough(D2,n) : door(D2);
opendoor(D3,n) : door(D3);
goto(O,n) : object(O);
callelevator(E,U,n) : elevator(E) , orientation(U); 
changefloor(R,n) : room(R)
}1 :- not noop(n).

noop(n) :- noop(n).