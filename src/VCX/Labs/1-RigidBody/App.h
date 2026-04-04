#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-RigidBody/SingleCaseBox.h"

#include "Labs/Common/UI.h"

namespace VCX::Labs::RigidBody {
    class App : public Engine::IApp {
    private:
        Common::UI _ui;

        SingleCaseBox        _singleCaseBox;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = { _singleCaseBox};

    public:
        App();

        void OnFrame() override;
    };
}
