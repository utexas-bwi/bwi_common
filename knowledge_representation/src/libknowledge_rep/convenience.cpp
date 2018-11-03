#include <knowledge_representation/convenience.h>
#ifdef USE_MYSQL
#include "mysql/LongTermMemoryConduit.h"
#elif USE_POSTGRESQL
#include "postgresql/LongTermMemoryConduit.h"
#endif

namespace knowledge_rep {
#ifdef USE_MYSQL
    LongTermMemoryConduit<LongTermMemoryConduitMySQL> get_default_ltmc() {
        return {"127.0.0.1", 33060, "root", "", "knowledge_base"};
    }
#elif USE_POSTGRESQL
    LongTermMemoryConduit<LongTermMemoryConduitPostgreSQL> get_default_ltmc() {
        return {};
    }
#endif
}
