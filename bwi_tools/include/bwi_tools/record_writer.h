#ifndef BWI_TOOLS_RECORD_WRITER_H
#define BWI_TOOLS_RECORD_WRITER_H

#include <map>
#include <string>
#include <vector>

#include <boost/algorithm/string/join.hpp>
#include <boost/foreach.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/copy.hpp>

namespace bwi_tools {
  
  inline bool writeRecordsAsCSV(const std::string &filename,
                                const std::vector<std::map<std::string, std::string> > &records) {

    // Get the superset of keys for all records.
    std::set<std::string> all_keys;
    typedef std::map<std::string, std::string> StringMap;
    BOOST_FOREACH(const StringMap& m, records) {
      boost::copy(m | boost::adaptors::map_keys, std::inserter(all_keys));
    }

    // Create the file and write the header
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
      return false;
    }
boost::algorithm::join(list, ",


  }

} /* bwi_tools */

#endif /* end of include guard: BWI_TOOLS_RECORD_WRITER_H */
