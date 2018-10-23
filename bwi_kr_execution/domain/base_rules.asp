#program base.

is_a(C1, C3) :- is_a(C1, C2), is_a(C2, C3).
instance_of_recursive(O, C1) :- instance_of(O, C2), is_a(C2, C1).
instance_of_recursive(O, C) :- instance_of(O, C).
has_concept(O, C) :- instance_of_recursive(O, C_id), name(C_id, C).

is_connected(L1, L3) :- is_connected(L1, L2), is_connected(L2, L3).

%% Support non-fluent to fluent promotion for basic attributes
is_near(self, L, 0) :- is_near(self, L).
is_in(self, R, 0) :- is_in(self, R).
is_near(P, L, 0) :- is_near(P, L).
is_open(D, 0) :- is_open(D).
is_placed(O, L, 0) :- is_placed(O, L).
is_facing(self, P, 0) :- is_facing(self, P).
is_holding(self, O, 0) :- is_holding(self, O).
is_delivered(O, P, 0) :- is_delivered(O, P).
is_near_name(self, L_name, 0) :- is_near(self, L_id), name(L_id, L_name).
is_placed_concept(O_concept, L_concept, 0) :- is_placed(O_id, L_id), has_concept(O_id, O_concept), has_concept(L_id, L_concept).
is_in_concept(O_concept, R_concept, 0) :- is_placed(O_id, L), is_in(L, R_id), has_concept(O_id, O_concept), has_concept(R_id, R_concept).
is_delivered_concept(O_concept, P, 0) :- is_delivered(O_id, P), has_concept(O_id, O_concept).
scanned_concept(L_concept, 0) :- scanned(L_id, 0), has_concept(L_id, L_concept).
is_holding_concept(self, O_concept, 0) :- is_holding(self, O_id, 0), has_concept(O_id, O_concept).

% short-hand for concepts
scanned(S, 0) :- has_concept(S, "scanned").
hand_empty(0) :- has_concept(self, "empty_handed").

% Silence warnings
default_location(O, L) :- default_location(O, L).
is_facing(self, P) :- is_facing(self, P).
is_near(self, L) :- is_near(self, L).
is_holding(self, O) :- is_holding(self, O).
is_in(L, R) :- is_in(L, R).
is_placed(O, L) :- is_placed(O, L).
is_near(P, L) :- is_near(P, L).
is_delivered(O, P) :- is_delivered(O, P).
-is_near(P, L, n) :- -is_near(P, L, n).
can_be_near(O, L, 0) :- can_be_near(O, L, 0).
can_be_placed(O, L, 0) :- can_be_placed(O, L, 0).
can_be_near(P, L) :- can_be_near(P, L, 0), has_concept(L, "beacon").
can_be_near(P, L) :- can_be_near(P, R, 0), has_concept(R, "room"), is_in(L, R), has_concept(L, "beacon").
can_be_placed(O, L) :- can_be_placed(O, L, 0), has_concept(L, "placement").
can_be_placed(O, L) :- can_be_placed(O, R, 0), has_concept(R, "room"), is_in(L, R), has_concept(L, "placement").
can_be_placed(O_id, L_id) :- default_location(O_category_id, L_concept_id), is_a(O_id, O_concept_id), is_a(O_concept_id, O_category_id), is_a(L_id, L_concept_id), has_concept(O_id, "hypothetical"), 0{can_be_placed(O_id, L, 0)}0.
