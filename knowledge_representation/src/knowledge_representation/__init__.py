from knowledge_representation._libknowledge_rep_wrapper_cpp import LongTermMemoryConduit, PyAttributeList, Entity, Concept, Instance
from knowledge_representation.memory_conduit import MemoryConduit

def get_default_ltmc():
    return LongTermMemoryConduit("127.0.0.1", 33060, "root", "", "knowledge_base")