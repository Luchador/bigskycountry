#pragma once

#include "pawn.hpp"

class GameWorld;

// One controller owns a pawn's commands. Player and AI share possession rules.
class PawnController
{
public:
    explicit PawnController(GameWorld& world) : world(world) {}
    virtual ~PawnController();
    PawnController(const PawnController&) = delete;
    PawnController& operator=(const PawnController&) = delete;
    virtual bool Possess(PawnHandle handle);
    void Release();
    Pawn* Controlled() const;
    virtual void Update(double) {}
    virtual bool SimulationReady() const { return true; }

protected:
    void Command(PawnInput input);
    GameWorld& world;

private:
    PawnHandle controlled = 0;
};
