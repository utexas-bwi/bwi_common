#ifndef bwi_actexec_GotoObject_h__guard
#define bwi_actexec_GotoObject_h__guard

#include "LogicalNavigation.h"

namespace bwi_krexec {

    class SenseLocation : public LogicalNavigation  {
    public:
        explicit SenseLocation();

        Action *cloneAndInit(const actasp::AspFluent & fluent) const override {
            return new SenseLocation();
        }

        Action *clone() const override {return new SenseLocation(*this);}

        int paramNumber() const override {return 0;};

        std::vector<std::string> getParameters() const override;

        std::vector<std::string> prepareGoalParameters() const override;

    };
}

#endif
