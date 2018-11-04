#pragma once
#include "LongTermMemoryConduitInterface.h"
#include <knowledge_representation/LTMCInstance.h>
#include <knowledge_representation/LTMCEntity.h>
#include <knowledge_representation/LTMCConcept.h>
#ifdef USE_MYSQL

#include "../../src/libknowledge_rep/mysql/LongTermMemoryConduitMySQL.h"
#elif USE_POSTGRESQL

#endif


namespace knowledge_rep {
#ifdef USE_MYSQL
typedef LTMCEntity<LongTermMemoryConduitMySQL> Entity;
typedef LTMCConcept<LongTermMemoryConduitMySQL> Concept;
typedef LTMCInstance<LongTermMemoryConduitMySQL> Instance;
typedef LongTermMemoryConduitMySQL LongTermMemoryConduit;
LongTermMemoryConduit get_default_ltmc();
#elif USE_POSTGRESQL
LongTermMemoryConduit<LongTermMemoryConduitPostgreSQL> get_default_ltmc() {
        return {};
    }
#endif
}
