

%you have to pick exactly one action

#cumulative n.

%LIST HERE ANY ACTION YOU ADD!!!

1{	approach(D1,I) : door(D1), 
	gothrough(D2,I) : door(D2), 
	opendoor(D3,I) : door(D3)
	}1 :- not noop(I), I=0..n-2.

%removes the warning about noop not being defined, shouldn't have any consequences
%noop(n).

#hide noop/1.

