#ifndef KNOWLEDGE_REPRESENTATION_COMPILER_H
#define KNOWLEDGE_REPRESENTATION_COMPILER_H
#include <knowledge_representation/Parser.h>
#include <knowledge_representation/location.hh>

#include <knowledge_representation/node.h>

namespace knowledge_rep {
class Lexer;
class Compiler
{
public:
  friend class Parser;
  friend class Lexer;

  bool trace_scanning;
  bool trace_parsing;
  Compiler();

  std::string file;

  void error(const location& l, const std::string& m);
  void error(const std::string& m);

  //vm::Script* compile(const std::string& text);

  bool parseString(const std::string& text);

  void set_root(Node& root);

  Node *get_root();

  bool parse_stream(std::istream &in);

  bool parse_file(const std::string &filename);

  bool parse_string(const std::string &input);


private:

  Node *root;
  std::vector<Node> m_commands;  // Example AST
  unsigned int m_location; // Used by scanner
  //Node* getRoot() { return root.get(); }
  Lexer *lexer;
};
}
#endif //KNOWLEDGE_REPRESENTATION_COMPILER_H
