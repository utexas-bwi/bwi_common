#ifndef BWI_KR_EXECUTION_BWIRESOURCEMANAGER_H
#define BWI_KR_EXECUTION_BWIRESOURCEMANAGER_H

#include <knowledge_representation/convenience.h>
#include <actasp/ResourceManager.h>

struct BwiResourceManager: public actasp::ResourceManager {

	BwiResourceManager() : ltmc(knowledge_rep::getDefaultLTMC()) {}

  knowledge_rep::LongTermMemoryConduit ltmc;
};

#endif //BWI_KR_EXECUTION_BWIRESOURCEMANAGER_H
