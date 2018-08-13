%{
#include <cstdio>
#include <iostream>
#include <knowledge_representation/node.h>
using namespace std;

// stuff from flex that bison needs to know about:
extern int yylex();
extern int yyparse();
extern FILE *yyin;
extern int line_num;
void yyerror(const char *s);


%}

// Bison fundamentally works by asking flex to get the next token, which it
// returns as an object of type "yystype".  But tokens could be of any
// arbitrary data type!  So we deal with that in Bison by defining a C union
// holding each of the types of tokens that Flex could return, and have Bison
// use that union instead of "int" for the definition of "yystype":

%union {
    Node *node;
    Query *query;
    FreeVar *var;
    Valuable *value;
    Idable *id;
    VId *vid;
    VFloat *vfloat;
    VBool *vbool;
    VString *vstring;
    AttributeName *attr_name;
    QueryId *q_id;
    QueryValue *q_value;
}

// define the constant-string tokens:



// define the "terminal symbol" token types I'm going to use (in CAPS
// by convention), and associate each with a field of the union:

%token FREE
%token <vid> INT
%token <vfloat> FLOAT
%token <vstring> STRING
%token <vbool> BOOL

%type <vid> id
%type <attr_name> attribute_name
%type <query> query
%type <value> valuable
%type <id> idable
%type <q_id> query_id
%type <q_value> query_value

%%

// the first rule defined is the highest-level rule, which in our

// case is just the concept of a whole "snazzle file":

program:
    queries { cout << "done" << endl; }
    ;
queries:
    queries query
    | query
    ;
valuable:
    STRING | BOOL | id | FLOAT
    | '(' query_value ')' | '(' query_id ')'
    ;
id:
    INT {$$ = $1;}
    ;
idable:
    id | '(' query_id ')'
    ;
attribute_name:
    STRING {$$ = new AttributeName($1->value);}
query:
    query_value | query_id
    | '(' query_value ')' | '(' query_id ')'
    ;
query_id:
    FREE attribute_name valuable { cout << "q- id:<free>\tattr:" << $2 << "\tvalue:" << $3 << endl; }
    ;
query_value:
    idable attribute_name FREE { cout << "q- id:" << $1 << "\tattr:" << $2 << "\tvalue:" << "<free>" << endl; }
    ;
%%


void yyerror(const char *s) {
    cout << "EEK, parse error on line " << line_num << "!  Message: " << s << endl;
    // might as well halt now:
    exit(-1);
}

