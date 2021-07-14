#ifndef bwi_actexec_GotoObject_h__guard
#define bwi_actexec_GotoObject_h__guard

#include "LogicalNavigation.h"
#include "../BwiResourceManager.h"

namespace bwi_krexec {

    class SenseLocation : public LogicalNavigation  {
    public:
        explicit SenseLocation(knowledge_rep::LongTermMemoryConduit &ltmc);

        Action *cloneAndInit(const actasp::AspFluent & fluent) const override {
            return nullptr;
        }

        Action *clone() const override {return nullptr;}

        int paramNumber() const override {return 0;};

        std::vector<std::string> getParameters() const override;

        boost::optional<std::vector<std::string> > prepareGoalParameters() const override;

        static std::unique_ptr<actasp::Action> create(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
          auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
          return std::unique_ptr<actasp::Action>(new bwi_krexec::SenseLocation(resource_manager_cast.ltmc));
        }

        static std::unique_ptr<actasp::Action> create(actasp::ResourceManager &resource_manager) {
          auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
          return std::unique_ptr<actasp::Action>(new bwi_krexec::SenseLocation(resource_manager_cast.ltmc));
        }

    };
}

#endif
