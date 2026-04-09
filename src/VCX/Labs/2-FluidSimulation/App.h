#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/2-FluidSimulation/FluidSimulator.h"


#include "Labs/Common/UI.h"

namespace VCX::Labs::Fluid {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        FluidSimulator _fluidSimulator;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _fluid };

    public:
        App();

        void OnFrame() override;
    };
}
