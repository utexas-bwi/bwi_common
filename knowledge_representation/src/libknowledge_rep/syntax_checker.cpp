#include <cstdio>
#include <iostream>
#include <cstring>

using namespace std;

// stuff from flex that bison needs to know about:
extern int yylex();
extern int yyparse();
extern FILE *yyin;
extern int line_num;


int main(int argc, char** argv) {

  if (argc == 3 && string(argv[1]) == string("-f")) {
    // open a file handle to a particular file:

    FILE *myfile = fopen(argv[2], "r");

    // make sure it's valid:

    if (!myfile) {

      cout << "I can't open the file!" << endl;

      return -1;

    }

    // Set flex to read from it instead of defaulting to STDIN:

    yyin = myfile;
  }



  // Parse through the input:

  yyparse();

}

