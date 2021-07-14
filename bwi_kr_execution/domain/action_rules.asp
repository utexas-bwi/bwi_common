#const self = 1.

#program step(n).

% action navigate_to
is_near(self, T, n) :- navigate_to(T, n).
is_facing(self, T, n) :- navigate_to(T, n).
is_in(self, R2, n) :- navigate_to(T, n), has_concept(T, "door"), has(R2, T), is_in(self, R1, n - 1), is_connected(R1, R2), R1 != R2.
is_in(self, R1, n) :- navigate_to(T, n), is_in(T, R1).
-is_near(self, F, n) :- navigate_to(T, n), is_near(self, F, n - 1), T != F.
:- navigate_to(T, n), is_in(T, R1), is_in(self, R2, n - 1), not is_connected(R1, R2).
:- navigate_to(T, n), has_concept(T, "door"), 0{has(R1, T); is_connected(R1, R2)}0, is_in(self, R2, n - 1).

% action open_door
is_open(D, n) :- open_door(D, n).
is_facing(self, D, n) :- open_door(D, n).
:- open_door(D, n), not is_facing(self, D, n - 1).

% action go_through
is_in(self, R2, n) :- go_through(D, n), is_in(self, R1, n - 1), has(R1, D), has(R2, D), R1 != R2.
:- go_through(D, n), not is_facing(self, D, n - 1).
:- go_through(D, n), not is_open(D, n - 1).

% action find_person
is_near(P, L, n) :- find_person(P, L, n).
is_facing(self, P, n) :- find_person(P, L, n).
:- find_person(P, L, n), not is_near(self, L, n - 1).
:- find_person(P, L, n), 0{can_be_near(P, L); is_near(P, L)}0.

% action pick_up
is_holding(self, O, n) :- pick_up(O, L, n).
-is_placed(O, L, n) :- pick_up(O, L, n).
-hand_empty(n) :- pick_up(O, L, n).
:- pick_up(O, L, n), not is_placed(O, L, n - 1).
:- pick_up(O, L, n), not is_facing(self, L, n - 1).
:- pick_up(O, L, n), not hand_empty(n - 1).
%:- pick_up(O, L, n), not has_concept(O, "sensed"), has_concept(L, "scanned").

% action put_down
-is_holding(self, O, n) :- put_down(O, L, n).
is_placed(O, L, n) :- put_down(O, L, n).
hand_empty(n) :- put_down(O, L, n).
:- put_down(O, L, n), not is_facing(self, L, n - 1).
:- put_down(O, L, n), not is_holding(self, O, n - 1).

% action perceive_surface
scanned(L, n) :- perceive_surface(O, L, n).
is_placed(O, L, n) :- perceive_surface(O, L, n).
is_facing(self, L, n) :- perceive_surface(O, L, n).
:- perceive_surface(O, L, n), not can_be_placed(O, L).
:- perceive_surface(O, L, n), not is_facing(self, L, n - 1).
:- perceive_surface(O, L, n), scanned(L, n - 1).

% action hand_over
is_delivered(O, P, n) :- hand_over(O, P, n).
hand_empty(n) :- hand_over(O, P, n).
:- hand_over(O, P, n), not is_facing(self, P, n - 1).
:- hand_over(O, P, n), not is_holding(self, O, n - 1).

%%static laws
-is_in(self, L1, n) :- is_in(self, L2, n), is_in(self, L1, n - 1), L1 != L2.

%%inertial rules
is_near(P, L, n) :- is_near(P, L, n - 1), not -is_near(P, L, n).
is_in(self, L, n) :- is_in(self, L, n - 1), not -is_in(self, L, n).
is_placed(O, L, n) :- is_placed(O, L, n - 1), not -is_placed(O, L, n).
scanned(L, n) :- scanned(L, n - 1).
is_holding(self, O, n) :- is_holding(self, O, n - 1), not -is_holding(self, O, n).
hand_empty(n) :- hand_empty(n - 1), not -hand_empty(n).
is_delivered(O, P, n) :- is_delivered(O, P, n - 1).

% allow passing concept name in goal query
is_near_name(self, L_name, n) :- is_near(self, L_id, n), name(L_id, L_name).
is_placed_concept(O_concept, L_concept, n) :- is_placed(O_id, L_id, n), has_concept(O_id, O_concept), has_concept(L_id, L_concept).
is_in_concept(O_concept, R_concept, n) :- is_placed(O_id, L, n), is_in(L, R_id), has_concept(O_id, O_concept), has_concept(R_id, R_concept).
is_delivered_concept(O_concept, P, n) :- is_delivered(O_id, P, n), has_concept(O_id, O_concept).
%scanned_concept(L_concept, n) :- scanned(L_id, n), has_concept(L_id, L_concept).
is_holding_concept(self, O_concept, n) :- is_holding(self, O_id, n), has_concept(O_id, O_concept).


