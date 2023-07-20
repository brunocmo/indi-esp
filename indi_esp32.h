#pragma once

#include "libindi/defaultdevice.h"
#include "inditelescope.h"
#include "libindi/connectionplugins/connectiontcp.h"
#include "libindi/alignment/AlignmentSubsystemForDrivers.h"

#include <tuple>

namespace Connection
{
    class TCP;
}

class Esp32Driver : 
    public INDI::Telescope,
    public INDI::AlignmentSubsystem::AlignmentSubsystemForDrivers
{
public:
    Esp32Driver();
    virtual ~Esp32Driver() = default;

    virtual bool initProperties() override;

    // You must override this method in your class.
    virtual const char *getDefaultName() override;

protected:
    bool Handshake();

    virtual bool ReadScopeStatus() override;
    virtual bool Goto(double, double) override;
    virtual bool Park() override;
    virtual bool Abort() override;
    bool updateLocation(double latitude, double longitude, double elevation) override;

private:
    double currentRA{13};
    double currentDEC{-89};
    double targetRA{0};
    double targetDEC{0};

    INDI::IHorizontalCoordinates trackingAltAz{ 0, 0 };

    bool sendCommand(const char *cmd);

    bool isAbort{ false };

    std::tuple< int, int > stepsNeededToMove( double, double );
    std::tuple< int, int > stepsTracking();

    Connection::TCP *tcpConnection{nullptr};
    // Debug channel to write mount logs to
    // Default INDI::Logger debugging/logging channel are Message, Warn, Error, and Debug
    // Since scope information can be _very_ verbose, we create another channel SCOPE specifically
    // for extra debug logs. This way the user can turn it on/off as desired.
    uint8_t DBG_SCOPE { INDI::Logger::DBG_IGNORE };

    // slew rate, degrees/s
    static const uint8_t SLEW_RATE = 15;
};
