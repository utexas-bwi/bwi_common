#ifndef BWI_TOOLS_FILESYSTEM_H
#define BWI_TOOLS_FILESYSTEM_H

#include <boost/filesystem.hpp>

namespace bwi {

  namespace fs {

    inline std::string canonicalize(const std::string& path) {
      boost::filesystem::path canonicalized_path = 
        boost::filesystem::absolute(
            boost::filesystem::path(path), 
            boost::filesystem::current_path());
      return canonicalized_path.normalize().string();
    }
    
  } /* fs */
  
} /* bwi */

#endif /* end of include guard: BWI_TOOLS_FILESYSTEM_H */
