#program base.
elevroom(l3test_404; l3_404).

room(R) :- elevroom(R). 

floor(l3test_404, 2).
floor(l3_404, 3).

sameroom(l3test_404, l3_404).

sameroom(R1,R2) :- sameroom(R2,R1). 
sameroom(R1,R2) :- sameroom(R1,R3), sameroom(R2,R3), room(R1), room(R2),
                   room(R3), R1!=R2, R2!=R3, R1!=R3. 

orientation(up).
orientation(down).

thing(D) :- door(D). 
thing(E) :- elevator(E).

elevator(l3_elev). 
elevator(l3test_elev). 

elevhasdoor(l3_elev, d3_404).
elevhasdoor(l3test_elev, d3test_404).

elevdoor(d3_404).
elevdoor(d3test_404).

door(D) :- elevdoor(D). 

hasdoor(l3_404, d3_404).
hasdoor(l3_400, d3_404).
hasdoor(l3test_404, d3test_404).
hasdoor(l3test_400, d3test_404).

%hide non fluents
%#hide elevdoor/2.
%#hide elevhasdoor/2.
%#hide elevator/1.
%#hide door/1.
%#hide thing/1.
%#hide sameroom/2.
%#hide orientation/1.
%#hide floor/2.
%#hide elevroom/1.
%#hide elevdoor/1.
