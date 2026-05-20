#pragma once

#include <vector>

#include "Engine/app.h"
#include "CaseFEMSoftBody.h"
#include "CaseCloth.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::FEM {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseFEMSoftBody _caseFEMSoftBody;
        CaseCloth       _caseCloth;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseFEMSoftBody, _caseCloth };

    public:
        App();

        void OnFrame() override;
    };
} // namespace VCX::Labs::FEM
