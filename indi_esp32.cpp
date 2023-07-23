#include "config.h"
#include "indi_esp32.h"
#include "indicom.h"
#include "libindi/libastro.h"


#include <cmath>
#include <memory>
#include <unistd.h>
#include <termios.h>
#include <iostream>
#include <sstream>

// We declare an auto pointer to MyCustomDriver.
static std::unique_ptr<Esp32Driver> esp32Driver(new Esp32Driver());

using namespace INDI::AlignmentSubsystem;

Esp32Driver::Esp32Driver()
{
    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
    DBG_SCOPE = INDI::Logger::getInstance().addDebugLevel("Scope Verbose", "SCOPE");
}

bool Esp32Driver::initProperties()
{
    // initialize the parent's properties first
    INDI::Telescope::initProperties();

    // Set telescope capabilities. 0 is for the the number of slew rates that we support. We have none for this simple driver.
    SetTelescopeCapability(TELESCOPE_CAN_GOTO | TELESCOPE_CAN_PARK | TELESCOPE_CAN_ABORT, 0);

    addAuxControls();

    // Add alignment properties
    InitAlignmentProperties(this);

    // Force the alignment system to always be on
    getSwitch("ALIGNMENT_SUBSYSTEM_ACTIVE")[0].setState(ISS_ON);

    return true;
}


bool Esp32Driver::Handshake()
{
    bool teste = false;
    if (isSimulation())
    {
        teste = updateLocation( -16.0209960, -48.0193535, 1163.4 );
        
        LOGF_INFO("Updated %d", teste);

        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());

        return true;
    }
    else
    {

        teste = updateLocation( -16.020996, -48.019353, 1163.4 );

        LOGF_INFO("Updated %d", PortFD );

        return true;
    }

    // TODO: Any initial communciation needed with our device; we have an active
    // connection with a valid file descriptor called PortFD. This file descriptor
    // can be used with the tty_* functions in indicom.h

    return true;
}

bool Esp32Driver::Goto( double ra, double dec )
{
    targetRA = ra;
    targetDEC = dec;

    const auto steps = stepsNeededToMove( ra, dec );

    LOGF_INFO("TESTE azimute: %d || altitude: %d ", std::get<0>(steps), std::get<1>(steps) );

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;

    char buffer[255];
    char* ptrBuffer = buffer;

    *ptrBuffer++ = 0x01;
    memcpy(ptrBuffer, &std::get<0>(steps), sizeof(int));
    ptrBuffer += sizeof(int);
    *ptrBuffer++ = ',';
    memcpy(ptrBuffer, &std::get<1>(steps), sizeof(int));
    ptrBuffer += sizeof(int);
    *ptrBuffer++ = '\0';

    // std::stringstream wow;
    // for( int i{0}; i<20; i++)
    // {
    //     wow << std::hex << (int)buffer[i] << " ";
    // }
    // LOGF_INFO( "%s", wow.str().c_str() );

    sendCommand( buffer );

    isAbort = false;

    return true;
}

bool Esp32Driver::Abort()
{
    char buffer[3];
    char* ptrBuffer = buffer;

    *ptrBuffer++ = 0x05;
    sendCommand( buffer );

    NewRaDec( currentRA, currentDEC );

    isAbort = true;
    azimuthAcc = 0;
    altitudeAcc = 0;

    return true;
}

bool Esp32Driver::Park()
{

    char buffer[3];
    char* ptrBuffer = buffer;

    INDI::IHorizontalCoordinates AltAz{ 180, 0 };
    INDI::IEquatorialCoordinates EquatorialCoordinates { 0, 0 };
    INDI::HorizontalToEquatorial(&AltAz, &m_Location, ln_get_julian_from_sys(), &EquatorialCoordinates);

    trackingAltAz = AltAz;
    TrackState = SCOPE_IDLE;

    currentRA = EquatorialCoordinates.rightascension;
    currentDEC = EquatorialCoordinates.declination;

    azimuthAcc = 0;
    altitudeAcc = 0;

    *ptrBuffer++ = 0x02;
    sendCommand( buffer );

    LOGF_INFO("TESTE ra: %f || dec: %f ", currentRA, currentDEC );

    NewRaDec( currentRA, currentDEC );

    return false;
}

bool Esp32Driver::ReadScopeStatus()
{
    static struct timeval ltv
    {
        0, 0
    };

    struct timeval tv
    {
        0, 0
    };
    double dt = 0, da_ra = 0, da_dec = 0, dx = 0, dy = 0;
    int nlocked;

    // update elapsed time since last poll, dont presume exatly POOLMS
    gettimeofday(&tv, nullptr);

    if ( ltv.tv_sec == 0 && ltv.tv_usec == 0 )
    {
        ltv = tv;
    }

    dt = tv.tv_sec - ltv.tv_sec + ( tv.tv_usec - ltv.tv_usec ) / 1e6;
    ltv = tv;

    // calculate how much we moved since last time
    da_ra = SLEW_RATE * dt;
    da_dec = SLEW_RATE * dt;

    // Process per current state. We check the state of EQUATORIAL_EOD_COORDS_REQUEST
    // and act acoordingly 
    switch (TrackState)
    /* condition */{
        case SCOPE_SLEWING:
        {
            // Wait until we are "locked" into position for both RA & DEC axis
            nlocked = 0;

            // Calculate diff in RA
            dx = targetRA - currentRA;

            // If diff is very small, i. e. smaller than how much we changed since
            // last time, the we reached target RA.
            if( fabs( dx ) * 15. <= da_ra )
            {
                currentRA = targetRA;
                nlocked++;
            }
            // Otherwise, increase RA
            else
            {
                dx > 0 ? ( currentRA += da_ra / 15.0 ) : ( currentRA -= da_ra / 15.0 );
            }

            // Calculate diff in DEC
            dy = targetDEC - currentDEC;

            // If diff is very small, i.e. smaller than how much we changeed since
            // last time, then we reached target DEC.
            if( fabs( dy ) <= da_dec )
            {
                currentDEC = targetDEC;
                nlocked++;
            }
            else
            {
                dy > 0 ? ( currentDEC += da_dec ) : ( currentDEC -= da_dec );
            }

            // lets check if we reached position for both RA/DEC
            if( nlocked == 2 )
            {
                // Lets set state to TRACKING
                TrackState = SCOPE_TRACKING;

                LOG_INFO("Telescope slew is complete. Tracking...");
            }
            break;
        }

        case SCOPE_TRACKING:
        {
            if( isAbort )
            {
                break;
            }
            const auto steps = stepsTracking();

            int stepsX = std::get<0>(steps);
            int stepsY = std::get<1>(steps);

            if( stepsX != 0 || stepsY != 0 )
            {
                LOGF_INFO("TESTE azimute: %d || altitude: %d ", stepsX, stepsY);
                char buffer[255];
                char* ptrBuffer = buffer;

                *ptrBuffer++ = 0x03;
                memcpy(ptrBuffer, &stepsX, sizeof(int));
                ptrBuffer += sizeof(int);
                *ptrBuffer++ = ',';
                memcpy(ptrBuffer, &stepsY, sizeof(int));
                ptrBuffer += sizeof(int);
                *ptrBuffer++ = '\0';
                sendCommand( buffer );
            }

            break;
        }
        default:
        break;
    } 
    
    char RAStr[64] = {0}, DecStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa( RAStr, currentRA, 2, 3600 );
    fs_sexa( DecStr, currentDEC, 2, 3600 );

    DEBUGF( DBG_SCOPE, "Current RA: %s Current DEC: %s", RAStr, DecStr );

    NewRaDec( currentRA, currentDEC );
    return true;
}

bool Esp32Driver::updateLocation(double latitude, double longitude, double elevation)
{
    m_Location.latitude = latitude;
    m_Location.longitude = longitude;
    m_Location.elevation = elevation;
    UpdateLocation(latitude, longitude, elevation);
    return true;
}

bool Esp32Driver::sendCommand(const char *cmd)
{
    int nbytes_read = 20;
    char res[20] = {0};
    LOGF_DEBUG("CMD <%s>", cmd);

    if (!isSimulation())
    {
        int n = write( PortFD, cmd, 255 );
        if ( n == -1 )
        {
            LOGF_ERROR("Write message error", cmd);
            return false;
        }
    }

    if (isSimulation())
    {
        strncpy(res, "OK#", 8);
        nbytes_read = 3;
    }
    else
    {
        int k = read( PortFD, res, 20);
        if ( k == -1 or k == 0 )
        {
            LOGF_ERROR("Read error", cmd );
            return false;
        }
        else
        {
            std::stringstream wow;
            for( int i{0}; i<20; i++)
            {
                wow << res[i];
            }
        }
    }

    res[nbytes_read - 1] = '\0';
    LOGF_DEBUG("RES <%s>", (char *)res);

    return true;
}

std::tuple< int, int > Esp32Driver::stepsNeededToMove( double ra, double dec )
{
    double diffAzimute{0};
    double diffAltitude{0};

    double anglePerStep = 0.00329670329;

    INDI::IHorizontalCoordinates currentAltAz{ 0, 0 };
    INDI::IHorizontalCoordinates positionAltAz{ 0, 0 };

    TelescopeDirectionVector currentTDV;
    TelescopeDirectionVector positionTDV;


    TransformCelestialToTelescope( currentRA, currentDEC, 0.0, currentTDV);
    TransformCelestialToTelescope( ra, dec, 0.0, positionTDV);

    AltitudeAzimuthFromTelescopeDirectionVector(currentTDV, currentAltAz);
    AltitudeAzimuthFromTelescopeDirectionVector(positionTDV, positionAltAz);

    trackingAltAz = positionAltAz;

    diffAzimute = positionAltAz.azimuth - currentAltAz.azimuth;
    diffAltitude = positionAltAz.altitude - currentAltAz.altitude;

    diffAzimute /= anglePerStep;
    diffAltitude /= anglePerStep;

    int stepsAzimute = (int)diffAzimute;
    azimuthAcc = diffAzimute - stepsAzimute;
    int stepsAltitude = (int)diffAltitude ;
    altitudeAcc = diffAltitude - stepsAltitude;

    return { stepsAzimute, stepsAltitude };
}

std::tuple< int, int > Esp32Driver::stepsTracking()
{
    double diffAzimute{0};
    double diffAltitude{0};

    double anglePerStep = 0.00329670329;

    INDI::IHorizontalCoordinates currentAltAz{ 0, 0 };

    TelescopeDirectionVector currentTDV;

    TransformCelestialToTelescope( currentRA, currentDEC, 0.0, currentTDV);

    AltitudeAzimuthFromTelescopeDirectionVector(currentTDV, currentAltAz);

    LOGF_INFO("tracking azimute: %lf || altitude: %lf ", trackingAltAz.azimuth, trackingAltAz.altitude );
    LOGF_INFO("current azimute: %lf || altitude: %lf ", currentAltAz.azimuth, currentAltAz.altitude );

    diffAzimute = currentAltAz.azimuth - trackingAltAz.azimuth;
    diffAltitude = currentAltAz.altitude - trackingAltAz.altitude;

    diffAzimute /= anglePerStep;
    diffAltitude /= anglePerStep;

    diffAzimute += azimuthAcc;
    diffAltitude += altitudeAcc;

    int stepsAzimute = (int)diffAzimute;
    azimuthAcc = diffAzimute - stepsAzimute;
    int stepsAltitude = (int)diffAltitude ;
    altitudeAcc = diffAltitude - stepsAltitude;

    if( stepsAzimute != 0 )
    {
        trackingAltAz.azimuth = currentAltAz.azimuth;
    }

    if( stepsAltitude != 0 )
    {
        trackingAltAz.altitude = currentAltAz.altitude;
    }

    return { stepsAzimute, stepsAltitude };
}

const char *Esp32Driver::getDefaultName()
{
    return "Esp32 Driver";
}

bool Esp32Driver::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    char buffer[255];
    char* ptrBuffer = buffer;

    *ptrBuffer++ = 0x01;

    int stepsFixed = 100;
    int stepsZeroed = 0;

    if( dir == 1 )
    {
        stepsFixed = -100;
    }

    memcpy(ptrBuffer, &stepsZeroed, sizeof(int));
    ptrBuffer += sizeof(int);
    *ptrBuffer++ = ',';
    memcpy(ptrBuffer, &stepsFixed, sizeof(int));
    ptrBuffer += sizeof(int);
    *ptrBuffer++ = '\0';

    sendCommand( buffer );

    INDI_UNUSED(command);
    IUResetSwitch(&MovementNSSP);
    MovementNSSP.s = IPS_IDLE;
    IDSetSwitch(&MovementNSSP, nullptr);
    return false;
}

bool Esp32Driver::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    char buffer[255];
    char* ptrBuffer = buffer;

    *ptrBuffer++ = 0x01;

    int stepsFixed = 100;
    int stepsZeroed = 0;

    if( dir == 0 )
    {
        stepsFixed = -100;
    }

    memcpy(ptrBuffer, &stepsFixed, sizeof(int));
    ptrBuffer += sizeof(int);
    *ptrBuffer++ = ',';
    memcpy(ptrBuffer, &stepsZeroed, sizeof(int));
    ptrBuffer += sizeof(int);
    *ptrBuffer++ = '\0';

    sendCommand( buffer );

    INDI_UNUSED(command);
    IUResetSwitch(&MovementWESP);
    MovementWESP.s = IPS_IDLE;
    IDSetSwitch(&MovementWESP, nullptr);
    return false;
}
