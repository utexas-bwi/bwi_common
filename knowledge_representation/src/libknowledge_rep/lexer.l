%{
#include <knowledge_representation/node.h>
#include "parser.tab.hpp"
int line_num = 1;
%}

%option noyywrap
%%
[ \t] ;
\?        { return FREE; }
[\(\)\{\}:;=,]    { return yytext[0]; }
[0-9]+\.[0-9]+ { yylval.vfloat = new VFloat(atof(yytext)); return FLOAT; }
[0-9]+         { yylval.vid = new VId(atoi(yytext)); return INT; }
true|false         { yylval.vbool = new VBool(atoi(yytext)); return BOOL; }
[a-zA-Z0-9]+   {
    // We have to strdup because we can't rely on yytext not changing underneath us:
    yylval.vstring = new VString(strdup(yytext));
    return STRING;
}
\n { ++line_num; }

%%