#include <knowledge_representation/convenience.h>

namespace knowledge_rep {
        LongTermMemoryConduit get_default_ltmc() {
            return LongTermMemoryConduit("127.0.0.1", 33060, "root", "", "knowledge_base");
    }
}
