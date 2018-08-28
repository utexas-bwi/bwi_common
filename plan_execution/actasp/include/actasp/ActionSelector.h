#pragma once

#include <actasp/AnswerSet.h>

#include <set>

namespace actasp {

struct ActionSelector  {

	virtual ActionSet::const_iterator choose(const ActionSet &options) noexcept = 0;

	virtual ~ActionSelector() {}
};
}


