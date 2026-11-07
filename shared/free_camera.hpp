#pragma once

struct CameraInput;

struct Vector3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

class FreeCamera
{
    public:
        void Update(const CameraInput& input, double elapsedSeconds);
        Vector3 WorldToView(Vector3 point) const;

    private:
        Vector3 Forward() const;
        Vector3 Right() const;

        Vector3 position;
        double yaw = 0.0;
        double pitch = 0.0;
};
