%skeleton "lalr1.cc"
%require "3.0.2"

%defines
%define api.namespace {knowledge_rep}
%define parser_class_name {Parser}

%define api.token.constructor
%define api.value.type variant
%define parse.assert true

// This will appear in Parser.h
%code requires {
  #include <knowledge_representation/node.h>

  namespace knowledge_rep
  {
    class Compiler;
    class Lexer;
  }
}

%lex-param { knowledge_rep::Lexer &lexer }
%lex-param { knowledge_rep::Compiler &compiler }
%parse-param { knowledge_rep::Lexer &lexer }
%parse-param { knowledge_rep::Compiler &compiler }

%locations
%initial-action
{
  @$.begin.filename = @$.end.filename = &compiler.file;
};

%define parse.trace
%define parse.error verbose

%code top {
    #include <cstdio>
    #include <iostream>
    #include <knowledge_representation/node.h>
    #include <knowledge_representation/Compiler.h>
    #include <knowledge_representation/Lexer.h>
    #include <knowledge_representation/Parser.h>


    using namespace std;
    namespace knowledge_rep {};

    static knowledge_rep::Parser::symbol_type yylex(knowledge_rep::Lexer &scanner, knowledge_rep::Compiler &compiler) {
        return scanner.get_next_token();
    };
}


// define the constant-string tokens:
%token FREE LPAREN RPAREN END

// define the "terminal symbol" token types I'm going to use (in CAPS
// by convention), and associate each with a field of the union:
%token <VId> INT
%token <VFloat> FLOAT
%token <VString> STRING
%token <VBool> BOOL

%type <VId> id
%type <AttributeName> attribute_name
%type <QueryId> query
%type <Valuable> valuable
%type <Idable> idable
%type <QueryId> query_id
%type <QueryValue> query_value
%type <Node> program
%%

// the first rule defined is the highest-level rule, which in our

// case is just the concept of a whole "snazzle file":

program:
    queries END { compiler.set_root($$); cout << "done" << endl; }
    ;
queries:
    queries query
    | query
    ;
valuable:
    STRING | BOOL | id | FLOAT
    | LPAREN query_value RPAREN | LPAREN query_id RPAREN
    ;
id:
    INT {$$ = $1;}
    ;
idable:
    id | LPAREN query_id RPAREN
    ;
attribute_name:
    STRING {$$ = AttributeName($1.value);}
query:
    query_value | query_id
    | LPAREN query_value RPAREN | LPAREN query_id RPAREN
    ;
query_id:
    FREE attribute_name valuable { cout << "q- id:<free>\tattr:" << $2.name << "\tvalue:" << "VALUABLE" << endl; }
    ;
query_value:
    idable attribute_name FREE { cout << "q- id:" << "IDABLE" << "\tattr:" << $2.name << "\tvalue:" << "<free>" << endl; }
    ;
%%


// Bison expects us to provide implementation - otherwise linker complains
void knowledge_rep::Parser::error(const location &loc , const std::string &message) {

        // Location should be initialized inside scanner action, but is not in this example.
        // Let's grab location directly from driver class.
	// cout << "Error: " << message << endl << "Location: " << loc << endl;

        //cout << "Error: " << message << endl << "Error location: " << driver.location() << endl;
}
