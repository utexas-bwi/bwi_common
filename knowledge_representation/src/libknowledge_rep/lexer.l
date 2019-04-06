%{
#include <knowledge_representation/Parser.h>
#include <knowledge_representation/Lexer.h>
#include <knowledge_representation/Compiler.h>
#include <knowledge_representation/node.h>

typedef knowledge_rep::Parser::token token;

//#define YY_USER_ACTION m_driver.increaseLocation(yyleng);
#define yyterminate() knowledge_rep::Parser::make_END(loc);

static knowledge_rep::location loc;

using namespace knowledge_rep;

int line_num = 1;
%}

%option nodefault
%option noyywrap
%option c++
%option yyclass="Lexer"
%%
[ \t] ;
\?        { return Parser::make_FREE(loc); }
\( {return Parser::make_LPAREN(loc);}
\) {return Parser::make_RPAREN(loc);}
[0-9]+\.[0-9]+ {  return Parser::make_FLOAT(atof(yytext), loc); }
[0-9]+         { return Parser::make_INT(atoi(yytext), loc); }
true|false         { return Parser::make_BOOL(atoi(yytext), loc); }
[a-zA-Z0-9]+   {
    // We have to strdup because we can't rely on yytext not changing underneath us:
    return Parser::make_STRING(std::string(strdup(yytext)), loc);
}
\n { ++line_num; }
<<EOF>> {std::cout << "ended of file" << std::endl; return yyterminate();}
%%