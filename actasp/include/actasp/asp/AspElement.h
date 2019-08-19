#pragma once

namespace actasp {
/***
 * @brief An interface type for ASP language constructs. Clingo wrappers have
 * expectations of implementors, including the ability to convert to a string.
 */
struct AspElement {
  virtual ~AspElement(){};
};

}