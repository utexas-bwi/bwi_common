#ifndef bwi_krexec_NavigateTo_h_guard
#define bwi_krexec_NavigateTo_h_guard

#include "LogicalNavigation.h"
#include "../BwiResourceManager.h"

namespace bwi_krexec {


    class NavigateTo : public LogicalNavigation {
    public:
        NavigateTo(int location_id, knowledge_rep::LongTermMemoryConduit &ltmc);

        void run() override;

        Action *cloneAndInit(const actasp::AspFluent & fluent) const override {
            return nullptr;
        }

        Action *clone() const override {return nullptr;}

        int paramNumber() const override {return 1;};

        std::vector<std::string> getParameters() const override;

        boost::optional<std::vector<std::string> > prepareGoalParameters() const;

        static std::unique_ptr<actasp::Action> create(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
          auto location = std::atoi(fluent.getParameters().at(0).c_str());
          auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
          return std::unique_ptr<actasp::Action>(new bwi_krexec::NavigateTo(location, resource_manager_cast.ltmc));
        }

    private:
        int location_id;

        void onFinished(bool success, const bwi_msgs::LogicalNavResult &result) override;
    };
}

#endif
