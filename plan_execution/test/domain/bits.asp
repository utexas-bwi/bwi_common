#include <incmode>.
#program step(n).

#const b = 10.

%%%%%%% Domain

bit(1..b).
not bit_on(1..b, 0).

%%%%%%% All actions
1{
turn_on(I, n) : bit(I);
turn_off(I, n) : bit(I)
}1.

%%%%%%% Actions

bit_on(I, n) :- turn_on(I, n).
not bit_on(I, n) :- turn_off(I, n).

%%%%%%%
#show turn_on/2.
#show turn_off/2.