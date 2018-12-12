#include <knowledge_representation/convenience.h>
#ifdef USE_MYSQL
#include "mysql/LongTermMemoryConduitMySQL.h"
#elif USE_POSTGRESQL
#include "postgresql/LongTermMemoryConduit.h"
#endif

namespace knowledge_rep {
#ifdef USE_MYSQL
    LongTermMemoryConduit get_default_ltmc() {
        return LongTermMemoryConduit(std::string("knowledge_base"));
    }
#elif USE_POSTGRESQL
    LongTermMemoryConduit<LongTermMemoryConduitPostgreSQL> get_default_ltmc() {
        return {};
    }
#endif
}
