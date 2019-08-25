#include <knowledge_representation/convenience.h>

namespace knowledge_rep {

    LongTermMemoryConduit get_default_ltmc() {
      std::string db_name = "knowledge_base";
      std::string db_pass = "nopass";
      if (const char *env_db_name = std::getenv("KNOWLEDGE_REP_DB_NAME")) {
        db_name = env_db_name;
      }
      if (const char *env_db_pass = std::getenv("KNOWLEDGE_REP_DB_PASSWORD")) {
        db_name = env_db_pass;
      }
      return LongTermMemoryConduit(db_name);
    }

}
