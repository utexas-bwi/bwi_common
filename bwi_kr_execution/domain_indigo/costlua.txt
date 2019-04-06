#begin_lua

loc_table={}
loc_table["cor"] = 0
loc_table["l3_414"] = 1
loc_table["l3_414a"] = 2
loc_table["l3_414b"] = 3
loc_table["l3_436"] = 4
loc_table["s3_516"] = 5
door_table={}
door_table["d3_502"] = 0
door_table["d3_508"] = 1
door_table["d3_510"] = 2
door_table["d3_512"] = 3
door_table["d3_516a"] = 4
door_table["d3_516b"] = 5
door_table["d3_436a"] = 6
door_table["d3_436b"] = 7
door_table["d3_428"] = 8
door_table["d3_426"] = 9
door_table["d3_414b1"] = 10
door_table["d3_414b2"] = 11
door_table["d3_414b3"] = 12
door_table["d3_414a3"] = 13
door_table["d3_414a2"] = 14
door_table["d3_414a1"] = 15
door_table["d3_412"] = 16
door_table["d3_402"] = 17
door_table["d3_404"] = 18
door_table["d3_416"] = 19
door_table["d3_418"] = 20
door_table["d3_420"] = 21
door_table["d3_422"] = 22
door_table["d3_430"] = 23
door_table["d3_432"] = 24
function dis(a,b,c)
	d1 = door_table[tostring(a)]
	d2 = door_table[tostring(b)]
	if d1==d2 then return 1 end
	loc = loc_table[tostring(c)]
	if loc==0 then
		if d1==0 then
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==1 then
			if d2==0 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==2 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==3 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==4 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==5 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==6 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==7 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==8 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==9 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==10 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==11 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==14 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==15 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==16 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==17 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==18 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==19 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==20 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==21 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==22 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==23 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==23 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==24 then return 1 end
		end
		if d1==24 then
			if d2==0 then return 1 end
			if d2==1 then return 1 end
			if d2==2 then return 1 end
			if d2==3 then return 1 end
			if d2==4 then return 1 end
			if d2==5 then return 1 end
			if d2==6 then return 1 end
			if d2==7 then return 1 end
			if d2==8 then return 1 end
			if d2==9 then return 1 end
			if d2==10 then return 1 end
			if d2==11 then return 1 end
			if d2==14 then return 1 end
			if d2==15 then return 1 end
			if d2==16 then return 1 end
			if d2==17 then return 1 end
			if d2==18 then return 1 end
			if d2==19 then return 1 end
			if d2==20 then return 1 end
			if d2==21 then return 1 end
			if d2==22 then return 1 end
			if d2==23 then return 1 end
		end
	end
	if loc==1 then
		if d1==12 then
			if d2==13 then return 1 end
		end
		if d1==13 then
			if d2==12 then return 1 end
		end
	end
	if loc==2 then
		if d1==13 then
			if d2==14 then return 1 end
			if d2==15 then return 1 end
		end
		if d1==14 then
			if d2==13 then return 1 end
			if d2==15 then return 1 end
		end
		if d1==15 then
			if d2==13 then return 1 end
			if d2==14 then return 1 end
		end
	end
	if loc==3 then
		if d1==10 then
			if d2==11 then return 1 end
			if d2==12 then return 1 end
		end
		if d1==11 then
			if d2==10 then return 1 end
			if d2==12 then return 1 end
		end
		if d1==12 then
			if d2==10 then return 1 end
			if d2==11 then return 1 end
		end
	end
	if loc==4 then
		if d1==6 then
			if d2==7 then return 1 end
		end
		if d1==7 then
			if d2==6 then return 1 end
		end
	end
	if loc==5 then
		if d1==4 then
			if d2==5 then return 1 end
		end
		if d1==5 then
			if d2==4 then return 1 end
		end
	end
	return 1
end

#end_lua.
