#pragma once

#include "imu_backend.h"

class Icm20948Backend final : public ImuBackend {
public:
    bool begin() override;
    bool update() override;
    bool hasGyroscope() const override { return true; }
};