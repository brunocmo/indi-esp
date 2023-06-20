#include "config.h"
#include "indi_esp32.h"
#include "indicom.h"
#include "libindi/libastro.h"


#include <cmath>
#include <memory>

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
    SetTelescopeCapability(TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT, 0);

    addAuxControls();

    // Add alignment properties
    InitAlignmentProperties(this);

    // Force the alignment system to always be on
    getSwitch("ALIGNMENT_SUBSYSTEM_ACTIVE")[0].setState(ISS_ON);

    tcpConnection = new Connection::TCP(this);
    tcpConnection->registerHandshake([&]() {return Handshake();});
    tcpConnection->setDefaultHost("192.168.0.184");
    tcpConnection->setDefaultPort(16188);
    registerConnection(tcpConnection);

    return true;
}

bool Esp32Driver::Handshake()
{
    if (isSimulation())
    {
        bool teste = false;
        teste = updateLocation( -15.7792, 47.9341, 1056.24 );
        
        LOGF_INFO("Updated %d", teste);

        LOGF_INFO("Connected successfuly to simulated %s.", getDeviceName());

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
    char RAStr[64] = {0}, DecStr[64] = {0};
    char ALTStr[64] = {0}, AZStr[64] = {0};

    // Parse the RA/DEC into strings
    fs_sexa(RAStr, targetRA, 2, 3600);
    fs_sexa(DecStr, targetDEC, 2, 3600);

    // Mark state as slewing
    TrackState = SCOPE_SLEWING;

    LOGF_INFO( "VALOR: %lf, %lf, %lf ", m_Location.latitude, m_Location.longitude, m_Location.elevation );

    INDI::IHorizontalCoordinates AltAz{ 0, 0 };
    TelescopeDirectionVector TDV;

    bool testekkk = false;
    testekkk = TransformCelestialToTelescope( ra, dec, 0.0, TDV);

    LOGF_INFO( "show %d", testekkk );

    AltitudeAzimuthFromTelescopeDirectionVector(TDV, AltAz);

    INDI::IEquatorialCoordinates EquatorialCoordinates { 0, 0 };
    //EquatorialCoordinates.rightascension = ra;
    //EquatorialCoordinates.declination = dec;
    //INDI::EquatorialToHorizontal( &EquatorialCoordinates, &m_Location, ln_get_julian_from_sys(), &AltAz);

    fs_sexa(ALTStr, AltAz.altitude, 2, 3600);
    fs_sexa(AZStr, AltAz.azimuth, 2, 3600);

    LOGF_INFO("Girando para RA: %s == %lf - DEC: %s == %lf ||| ALT: %s = %lf - AZ: %s = %lf", RAStr, ra, DecStr, dec, ALTStr, AltAz.altitude, AZStr, AltAz.azimuth );

    return true;
}

bool Esp32Driver::Abort()
{
    return true;
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

const char *Esp32Driver::getDefaultName()
{

    return "Esp32 Driver";
}
