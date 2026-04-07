#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/CaseSingleBody.h"
#include "Labs/1-RigidBody/CaseTwoBodies.h"
#include "Labs/1-RigidBody/CaseComplexScene.h"
#include "Labs/1-RigidBody/CaseNewtonPendulum.h"

#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        CaseSingleBody _caseSingleBody;

        CaseTwoBodies _caseTwoBodies;

        CaseComplexScene _caseComplexScene;

        CaseNewtonPendulum _caseNewtonPendulum;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _caseSingleBody, _caseTwoBodies, _caseComplexScene, _caseNewtonPendulum };

    public:
        App();

        void OnFrame() override;
    };
}
