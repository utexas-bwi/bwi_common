#pragma once

#include <actasp/asp/AspElement.h>
namespace actasp {

/**
 * @brief A convenience wrapper for AspRule that only has a head.
 * Ex: is_robot(1), another_atom("some string value").
 */
class AspFact : public AspRule, public AspElement {
  AspLaw(std::vector<AtomType> head) : head(head), body() {}

};

}