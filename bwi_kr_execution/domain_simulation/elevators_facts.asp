#program base.

elevroom(l2_elev_east; l3_elev_east).
elevroom(l2_elev_west; l3_elev_west).

room(R) :- elevroom(R).

floor(l2_elev_east, 2).
floor(l2_elev_west, 2).
floor(l3_elev_east, 3).
floor(l3_elev_west, 3).

sameroom(l2_elev_east, l3_elev_east).
sameroom(l2_elev_west, l3_elev_west).

sameroom(R1,R2) :- sameroom(R2,R1).
sameroom(R1,R2) :- sameroom(R1,R3), sameroom(R2,R3), room(R1), room(R2),
                   room(R3), R1!=R2, R2!=R3, R1!=R3.

orientation(up).
orientation(down).

thing(D) :- door(D).
thing(E) :- elevator(E).

elevator(l3_elev).
elevator(l2_elev).

elevhasdoor(l3_elev, d3_elev_east).
elevhasdoor(l3_elev, d3_elev_west).
elevhasdoor(l2_elev, d2_elev_east).
elevhasdoor(l2_elev, d2_elev_west).

elevdoor(d3_elev_east).
elevdoor(d3_elev_west).
elevdoor(d2_elev_east).
elevdoor(d2_elev_west).

door(D) :- elevdoor(D).

hasdoor(l2_elev_east, d2_elev_east).
hasdoor(l2_elev_west, d2_elev_west).
hasdoor(l2_200, d2_elev_east).
hasdoor(l2_200, d2_elev_west).

hasdoor(l3_elev_east, d3_elev_east).
hasdoor(l3_elev_west, d3_elev_west).
hasdoor(l3_200, d3_elev_east).
hasdoor(l3_200, d3_elev_west).

%#show hasdoor/2.
%#show elevhasdoor/2.
%#show elevator/1.
%#show door/1.
%#show thing/1.
%#show sameroom/2.
%#show orientation/1.
%#show floor/2.
%#show elevroom/1.
%#show elevdoor/1.
