#pragma once

#include <actasp/asp/AspElement.h>
#include <string>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/variant.hpp>
#include <clingo.hh>

namespace actasp {

// The "ASP-Core-2 Input Language Format" paper lays out the ASP grammar in detail
// https://www.mat.unical.it/aspcomp2013/files/ASP-CORE-2.03c.pdf

/***
 * Terms are simplest the building blocks of ASP programs. Mostly, they're used
 * as arguments to atoms (AKA AspFunctions that appear in the the head of rules or as facts).
 */
struct AspTerm : virtual public AspElement {
  virtual std::string to_string() const = 0;

  /***
   * Returns a pointer to a copy of the object on the heap.
   * We need this so we can cleanly support boost::ptr_container
   * @return
   */
  virtual AspTerm* clone() const = 0;

  virtual bool isEqual(const AspTerm &other) const =0;

  virtual bool isLess(const AspTerm &other) const =0;

  /***
   * ASP defines a total ordering over all terms. Each term type returns it's its intrinsic
   * position in this ordering. A more expensive isEqual comparison is required to verify
   * that two instances that return the same index are actually the same.
   * @return
   */
  virtual int totalOrderIndex() const = 0;

  static std::unique_ptr<AspTerm> from_ast(const Clingo::AST::Term &term);

  static std::unique_ptr<AspTerm> from_symbol(const Clingo::Symbol &symbol);
};

/***
 * We implement this hook so that boost:ptr_container knows how to copy heap objects
 * @param object
 * @return
 */
inline AspTerm* new_clone(const AspTerm& object) {
  return object.clone();
};

typedef boost::ptr_vector<AspTerm> TermContainer;

inline TermContainer term_to_container(const AspTerm &term) {
  TermContainer elements;
  elements.push_back(term.clone());
  return elements;
}

inline TermContainer ast_terms_to_container(const std::vector<Clingo::AST::Term> &terms) {
  TermContainer elements;
  for (const auto &ast_term: terms) {
    elements.push_back(AspTerm::from_ast(ast_term)->clone());
  }
  return elements;
}



}