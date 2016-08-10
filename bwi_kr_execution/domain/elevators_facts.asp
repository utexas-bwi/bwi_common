#program base.

elevroom(l4_elev_east).
elevroom(l4_elev_west).
elevroom(l3_elev_east).
elevroom(l3_elev_west).
elevroom(l2_elev_east).
elevroom(l2_elev_west). 
elevroom(l1_elev_east).
elevroom(l1_elev_west).

elevroom(l4_elev_south).
elevroom(l3_elev_south).
elevroom(l2_elev_south).
elevroom(l1_elev_south).

room(R) :- elevroom(R). 

floor(l4_elev_east, 4).
floor(l4_elev_west, 4).
floor(l3_elev_east, 3).
floor(l3_elev_west, 3).
floor(l2_elev_east, 2).
floor(l2_elev_west, 2).
floor(l1_elev_east, 1).
floor(l1_elev_west, 1).

floor(l4_elev_south, 4).
floor(l3_elev_south, 3).
floor(l2_elev_south, 2).
floor(l1_elev_south, 1).


sameroom(l4_elev_east, l3_elev_east).
sameroom(l3_elev_east, l2_elev_east).
sameroom(l2_elev_east, l1_elev_east).

sameroom(l4_elev_west, l3_elev_west).
sameroom(l3_elev_west, l2_elev_west).
sameroom(l2_elev_west, l1_elev_west).

sameroom(l4_elev_south, l3_elev_south).
sameroom(l3_elev_south, l2_elev_south).
sameroom(l2_elev_south, l1_elev_south).

sameroom(R1,R2) :- sameroom(R2,R1). 
sameroom(R1,R2) :- sameroom(R1,R3), sameroom(R2,R3), room(R1), room(R2),
                   room(R3), R1!=R2, R2!=R3, R1!=R3. 

orientation(up).
orientation(down).

thing(D) :- door(D). 
thing(E) :- elevator(E).

elevator(l4_elev).
elevator(l3_elev). 
elevator(l2_elev). 
elevator(l1_elev).

elevhasdoor(l4_elev, d4_elev_east).
elevhasdoor(l4_elev, d4_elev_west).
elevhasdoor(l3_elev, d3_elev_east).
elevhasdoor(l3_elev, d3_elev_west).
elevhasdoor(l2_elev, d2_elev_east).
elevhasdoor(l2_elev, d2_elev_west).
elevhasdoor(l1_elev, d1_elev_east).
elevhasdoor(l1_elev, d1_elev_west).

%3rd Floor South Additions
elevhasdoor(l4_elev, d4_elev_south).
elevhasdoor(l3_elev, d3_elev_south).
elevhasdoor(l2_elev, d2_elev_south).
elevhasdoor(l1_elev, d1_elev_south).

elevdoor(d4_elev_east).
elevdoor(d4_elev_west).
elevdoor(d3_elev_east).
elevdoor(d3_elev_west).
elevdoor(d2_elev_east).
elevdoor(d2_elev_west).
elevdoor(d1_elev_east).
elevdoor(d1_elev_west).

elevdoor(d4_elev_south).
elevdoor(d3_elev_south).
elevdoor(d2_elev_south).
elevdoor(d1_elev_south).

door(D) :- elevdoor(D). 

hasdoor(l1_elev_east, d1_elev_east).
hasdoor(l1_elev_west, d1_elev_west).
hasdoor(l1_200, d1_elev_east).
hasdoor(l1_200, d1_elev_west).

hasdoor(l2_elev_east, d2_elev_east).
hasdoor(l2_elev_west, d2_elev_west).
hasdoor(l2_200, d2_elev_east).
hasdoor(l2_200, d2_elev_west).

hasdoor(l3_elev_east, d3_elev_east).
hasdoor(l3_elev_west, d3_elev_west).
hasdoor(l3_200, d3_elev_east).
hasdoor(l3_200, d3_elev_west).

hasdoor(l4_elev_east, d4_elev_east).
hasdoor(l4_elev_west, d4_elev_west).
hasdoor(l4_200, d4_elev_east).
hasdoor(l4_200, d4_elev_west).

hasdoor(l1_elev_south, d1_elev_south).
hasdoor(l1_600, d1_elev_south).

hasdoor(l2_elev_south, d2_elev_south).
hasdoor(l2_600, d2_elev_south).

hasdoor(l3_elev_south, d3_elev_south).
hasdoor(l3_600, d3_elev_south).

hasdoor(l4_elev_south, d4_elev_south).
hasdoor(l4_600, d4_elev_south).


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
