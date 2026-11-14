#pragma once

#include "camera_view.hpp"

struct CameraInput;

class FreeCamera : public CameraView
{
public:
    void Update(const CameraInput& input, double elapsedSeconds);
    double GetMoveSpeed() const { return moveSpeed; }
    double GetSpeedFraction() const;
    void SetSpeedFraction(double fraction);
    void ScrollSpeed(double wheelSteps);

private:
    double moveSpeed = 100.0; // m/s
};
