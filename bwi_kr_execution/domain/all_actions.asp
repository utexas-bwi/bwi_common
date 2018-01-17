#program cumulative(n).

1{
approach(D1,I) : door(D1);
gothrough(D2,I) : door(D2);
opendoor(D3,I) : door(D3);
goto(O,I) : object(O);
callelevator(E,U,I) : elevator(E) , orientation(U); 
changefloor(R,I) : room(R);
knock(D,I) : door(D);
searchroom(P,R,I): person(P), room(R);
delivermessage(P,M,I): message(P,M,I)
%askperson(P1,P2,I): person(P1) , person(P2);
%remind(P3,M,R2,I) : person(P3) , meeting(M,G,R2) , room(R2)
}1 :- not noop(I), I>0, I=n-1.

noop(I) :- noop(I), I>0, I=n-1.

#show approach/2.
#show gothrough/2.
#show opendoor/2.
#show goto/2.
#show callelevator/3.
#show changefloor/2.
#show knock/2.
#show searchroom/3.
#show delivermessage/3.
