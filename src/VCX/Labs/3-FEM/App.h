#pragma once

#include <vector>

#include "Engine/app.h"
#include "CaseFEMSoftBody.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFEMSoftBody _caseFEMSoftBody;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFEMSoftBody };

    public:
        App();

        void OnFrame() override;
    };
} // namespace VCX::Labs::FEM
