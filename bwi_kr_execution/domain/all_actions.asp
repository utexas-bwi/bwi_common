#cumulative n.

%you have to pick exactly one action

%LIST HERE ANY ACTION YOU ADD!!!

1{
  approach(D1,I) : door(D1), 
  gothrough(D2,I) : door(D2), 
  opendoor(D3,I) : door(D3), 
  goto(O,I) : object(O),
  callelevator(E,U,I) : elevator(E) : orientation(U), 
  changefloor(R,I) : room(R),
  searchroom(P,R1,I): person(P) : room(R1),
  askperson(P1,P2,I): person(P1) : person(P2),
  remind(P3,M,R2,I) : person(P3) : meeting(M,G,R2) : room(R2)
}1 :- not noop(I), I=0..n-2.


%removes the warning about noop not being defined, shouldn't have any consequences

%iclingo doesn't seem to like this, commenting.
%#volatile n.
noop(n) :- noop(n).

% #hide noop/1.

