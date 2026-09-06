#pragma once

#include "imu_backend.h"

class Lsm303Lis2mdlBackend final : public ImuBackend {
public:
    bool begin() override;
    bool update() override;
    bool hasGyroscope() const override { return false; }
};