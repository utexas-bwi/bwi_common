#include "ActionLogger.h"

#include "actasp/AspFluent.h"


#include <ctime>

using namespace std;
using namespace actasp;

ActionLogger::ActionLogger() : dest_file(NULL){}


void ActionLogger::setFile(const std::string& path) {
  taskCompleted();
  dest_file = new ofstream(path.c_str() , std::ofstream::app);
  time_t rawtime;
  struct tm * timeinfo;
  char time_string[10];
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (time_string,10,"%R",timeinfo);
  (*dest_file) << time_string << " "; 
  
}


void ActionLogger::actionStarted(const AspFluent& action) throw() {
 
  if(dest_file) {
    (*dest_file) << action.toString() << " ";
  }
}

void ActionLogger::actionTerminated(const AspFluent& action) throw()  {}

ActionLogger::~ActionLogger() {
 taskCompleted();
}

void ActionLogger::taskCompleted() {
  if(dest_file) {
    (*dest_file) << endl;
    dest_file->close();
    delete dest_file;
    dest_file = NULL;
  }
}

