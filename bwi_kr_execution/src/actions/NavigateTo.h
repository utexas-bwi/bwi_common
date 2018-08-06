#ifndef bwi_krexec_NavigateTo_h_guard
#define bwi_krexec_NavigateTo_h_guard

#include "LogicalNavigation.h"

namespace bwi_krexec {


    class NavigateTo : public LogicalNavigation {
    public:
        explicit NavigateTo();

        void run() override;

        Action *cloneAndInit(const actasp::AspFluent & fluent) const override {
            auto action = new NavigateTo();
            action->location_id = std::atoi(fluent.getParameters().at(0).c_str());
            action->ltmc = this->ltmc;
            return action;
        }

        Action *clone() const override {return new NavigateTo(*this);}

        int paramNumber() const override {return 1;};

        std::vector<std::string> getParameters() const override;

        std::vector<std::string> prepareGoalParameters() const;

    private:
        int location_id;
    };
}

#endif
