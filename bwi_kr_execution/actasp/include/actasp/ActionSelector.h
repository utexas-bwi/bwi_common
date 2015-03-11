#ifndef actasp_ActionSelector_h__guard
#define actasp_ActionSelector_h__guard

#include <actasp/AnswerSet.h>

#include <set>

namespace actasp {

struct ActionSelector  {

	virtual ActionSet::const_iterator choose(const ActionSet &options) throw() = 0;

	virtual ~ActionSelector() {}
};
}

#endif
