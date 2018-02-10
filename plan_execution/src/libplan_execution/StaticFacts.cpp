#include "plan_execution/StaticFacts.h"

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <fstream>

using namespace std;
using namespace actasp;

namespace plan_exec {

std::list<actasp::AspAtom> StaticFacts::static_facts;

void StaticFacts::retrieveStaticFacts(AspKR *reasoner, const std::string& domain_directory) {
  // Construct the query by showing everything. From the ASP files, collect every hide statement and convert it to a 
  // show statement. Then make a generic query.
  /* std::cout << "in" << std::endl; */
  std::vector<std::string> query_string_list;
  if (boost::filesystem::exists(domain_directory)) {
    boost::filesystem::directory_iterator itr(domain_directory);
    boost::filesystem::directory_iterator end_itr;
    for (; itr != end_itr; ++itr) {
      if (boost::algorithm::contains(itr->path().string(), ".asp")) {
        // Read this file and collect all lines with #show on them.
        std::ifstream infile(itr->path().string().c_str());
        std::string line;
        while (std::getline(infile, line)) {
          if (boost::algorithm::contains(line, "%#show")) {
            boost::replace_all(line, "%", "");
            query_string_list.push_back(line);
          }
        }
        infile.close();
      }
    }
  }

  /* std::cout << "out" << std::endl; */
  std::string query = boost::algorithm::join(query_string_list, "\n");
  static_facts = (reasoner->query(query, 0)).front();
}

std::list<actasp::AspAtom> StaticFacts::staticFacts() {
  return static_facts;
}
	
}
