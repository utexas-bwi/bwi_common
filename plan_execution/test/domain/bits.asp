#include <incmode>.
#program base.
#const b = 2.

#const incvar = "n".

%%%%%%% Domain

bit(1..b).
-bit_on(1..b, 0).

% Domain annotations that will help plan_execution process answer sets
action("turn_on";"turn_off";"all_on";"all_off").
fluent("bit_on").

#program step(n).

%%%%%%% All actions
1{
turn_on(I, n): bit(I);
turn_off(I, n): bit(I);
all_on(n);
all_off(n)
}1.

%%%%%%% Actions

bit_on(I, n) :- turn_on(I, n).

-bit_on(I, n) :- turn_off(I, n).

bit_on(1..b, n) :- all_on(n).

-bit_on(1..b, n) :- all_off(n).

%%%%%%% Inertial

bit_on(I, n) :- bit_on(I, n-1), not turn_off(I, n), not all_off(n).
-bit_on(I, n) :- -bit_on(I, n-1), not turn_on(I, n), not all_on(n).

%%%%%%%
