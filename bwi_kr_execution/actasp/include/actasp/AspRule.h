#ifndef actasp_AspRule_h__guard
#define actasp_AspRule_h__guard

#include <actasp/AspFluent.h>

#include <vector>

namespace actasp {


struct AspRule {
	
	AspRule() : head(), body() {}
	
	//implicit cast from fluent to rule
	AspRule(const AspFluent& fluent) throw() {
		body.push_back(fluent);
	}
	
	AspRule& operator<< (AspFluent fluent) throw (){
		body.push_back(fluent);
		return *this;
	}
	
	std::vector<AspFluent> head;
	std::vector<AspFluent> body;

};
	
}
#endif
