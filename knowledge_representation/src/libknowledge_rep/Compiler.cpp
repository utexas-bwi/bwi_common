#include <knowledge_representation/Compiler.h>
#include <sstream>
#include <fstream>
#include <knowledge_representation/Lexer.h>

namespace knowledge_rep {
  Compiler::Compiler() {

  }

  bool Compiler::parse_stream(std::istream &in) {
    Lexer lexer(*this, &in);
    lexer.set_debug(trace_scanning);
    this->lexer = &lexer;

    Parser parser(lexer, *this);
    parser.set_debug_level(trace_parsing);
    return (parser.parse() == 0);
  }

  bool Compiler::parse_file(const std::string &filename) {
    std::ifstream in(filename.c_str());
    if (!in.good()) return false;
    return parse_stream(in);
  }

  bool Compiler::parse_string(const std::string &input) {
    std::istringstream iss(input);
    return parse_stream(iss);
  }

void Compiler::set_root(Node &root) {
    this->root = &root;

}

Node *Compiler::get_root() {
    return root;
}

}