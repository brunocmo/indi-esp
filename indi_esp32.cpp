#include "config.h"
#include "indi_esp32.h"

// We declare an auto pointer to MyCustomDriver.
static std::unique_ptr<Esp32Driver> mydriver(new Esp32Driver());

Esp32Driver::Esp32Driver()
{
    setVersion(CDRIVER_VERSION_MAJOR, CDRIVER_VERSION_MINOR);
}

bool Esp32Driver::initProperties()
{
    // initialize the parent's properties first
    INDI::DefaultDevice::initProperties();

    SayHelloSP[SAY_HELLO_DEFAULT].fill(
        "SAY_HELLO_DEFAULT",    // The name of the VALUE
        "Say Hello",            // The label of the VALUE
        ISS_OFF                 // The switch state
        );
        
    SayHelloSP[SAY_HELLO_CUSTOM].fill(
        "SAY_HELLO_CUSTOM",     // The name of the VALUE
        "Say Custom",           // The label of the VALUE
        ISS_OFF                 // The switch state
        );
    
    SayHelloSP.fill(
        getDeviceName(),  // The name of the device
        "SAY_HELLO",      // The name of the PROPERTY
        "Hello Commands", // The label of the PROPERTY
        MAIN_CONTROL_TAB, // What tab should we be on?
        IP_RW,            // Let's make it read/write.
        ISR_ATMOST1,      // At most 1 can be on
        60,               // With a timeout of 60 seconds
        IPS_IDLE          // and an initial state of idle.
    );      
    // now we register the property with the DefaultDevice
    // without this, the property won't show up on the control panel
    // NOTE: you don't have to call defineProperty here. You can call it at
    // any time. Maybe you don't want it to show until you are connected, or
    // until the user does something else? Maybe you want to connect, query your
    // device, then call this. It's up to you.
    defineProperty(&SayHelloSP);

    WhatToSayTP[0].fill("WHAT_TO_SAY", "What to say?", "Hello, world!");
    WhatToSayTP.fill(getDeviceName(), "WHAT_TO_SAY", "Got something to say?", MAIN_CONTROL_TAB, IP_RW, 60, IPS_IDLE);
    defineProperty(&WhatToSayTP);

        // and now let's add a counter of how many times the user clicks the button
    SayCountNP[0].fill(
                 "SAY_COUNT",       // name of the VALUE
                 "Count",           // label of the VALUE
                 "%0.f",            // format specifier to show the value to the user; this should be a format specifier for a double
                 0,                 // minimum value; used by the client to render the UI
                 0,                 // maximum value; used by the client to render the UI
                 0,                 // step value; used by the client to render the UI
                 0);                // current value

    SayCountNP.fill(
                getDeviceName(),    // device name
                "SAY_COUNT",        // PROPERTY name
                "Say Count",        // PROPERTY label
                MAIN_CONTROL_TAB,   // What tab should we be on?
                IP_RO,              // Make this read-only
                0,                  // With no timeout
                IPS_IDLE);          // and an initial state of idle

    return true;

}

bool Esp32Driver::updateProperties()
{
    INDI::DefaultDevice::updateProperties();

    LOG_INFO("Teste esp32 update!");

    if (isConnected())
    {
        defineProperty(&SayCountNP);

    }
    else
    {
        deleteProperty(SayCountNP.getName());
    }



    return true;
}

bool Esp32Driver::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[],
                                 int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (SayHelloSP.isNameMatch(name))
        {
            // Accept what we received.
            SayHelloSP.update(states, names, n);

            // Find out what switch was clicked.
            int index = SayHelloSP.findOnSwitchIndex();
            switch (index)
            {
            case SAY_HELLO_DEFAULT: // see how much better this is than direct indexes? USE AN ENUM!
                LOG_INFO("Hello, world!");
                break;
            case SAY_HELLO_CUSTOM:
                LOG_INFO(WhatToSayTP[0].getText());
                break;
            }

            // Turn all switches back off.
            SayHelloSP.reset();

            // Set the property state back to idle
            SayHelloSP.setState(IPS_IDLE);

            // And actually inform INDI of those two operations
            SayHelloSP.apply();

            // Increment our "Say Count" counter.
            // Here we update the value on the property.
            SayCountNP[0].setValue(SayCountNP[0].getValue() + 1);
            // And then send a message to the clients to let them know it is updated.
            SayCountNP.apply();

            return true;
        }
    }

    // Nobody has claimed this, so let the parent handle it
    return INDI::DefaultDevice::ISNewSwitch(dev, name, states, names, n);
}

bool Esp32Driver::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    // Make sure it is for us.
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        if (WhatToSayTP.isNameMatch(name))
        {
            LOG_INFO(WhatToSayTP[0].getText());
            // Update the property to what the client sent
            // All elements in the property will now by synced with the client.
            WhatToSayTP.update(texts, names, n);
            // Set state to Idle
            WhatToSayTP.setState(IPS_IDLE);
            // Send back to client.
            WhatToSayTP.apply();
            return true;
        }  if (SayHelloSP.isNameMatch(name))
        {
            // Log a message. This will show up in the control panel.
            LOG_INFO("Hello, world!");

            // Turn the switch back off
            SayHelloSP[SAY_HELLO_DEFAULT].setState(ISS_OFF);

            // Set the property state back to idle
            SayHelloSP.setState(IPS_IDLE);

            // And actually inform INDI of those two operations
            SayHelloSP.apply();
            
            // We're done!
            return true;
        }
    }

    // Nobody has claimed this, so let the parent handle it
    return INDI::DefaultDevice::ISNewText(dev, name, texts, names, n);
}

bool Esp32Driver::saveConfigItems(FILE *fp)
{
    INDI::DefaultDevice::saveConfigItems(fp);
    IUSaveConfigText(fp, &WhatToSayTP);

    return true;
}



const char *Esp32Driver::getDefaultName()
{

    return "My Custom Driver";
}
