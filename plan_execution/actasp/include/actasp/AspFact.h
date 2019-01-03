#pragma once

namespace actasp {

template<typename AtomType>
class AspFact : public AspLaw<AtomType> {
  AspLaw(std::vector<AtomType> head) : head(head), body() {}

};

}