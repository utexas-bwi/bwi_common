#ifndef KNOWLEDGE_REPRESENTATION_LEXER_H
#define KNOWLEDGE_REPRESENTATION_LEXER_H


#undef YY_DECL
#define YY_DECL knowledge_rep::Parser::symbol_type knowledge_rep::Lexer::get_next_token()

#if !defined(yyFlexLexerOnce)
#include <FlexLexer.h>
#endif
#include <knowledge_representation/Parser.h>

namespace knowledge_rep
{
class Compiler;

class Lexer : public yyFlexLexer
{
public:

  Lexer(Compiler &compiler, std::istream *in) : yyFlexLexer(in), compiler(compiler) {}

  virtual knowledge_rep::Parser::symbol_type get_next_token();
  virtual ~Lexer() { }

private:

  Compiler &compiler;
};

}
#endif