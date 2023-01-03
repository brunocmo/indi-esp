#pragma once

#include "libindi/defaultdevice.h"
#include "libindi/indipropertyswitch.h"
#include "libindi/indipropertytext.h"

class Esp32Driver : public INDI::DefaultDevice
{
public:
    Esp32Driver();
    virtual ~Esp32Driver() = default;

    virtual bool initProperties() override;

    // You must override this method in your class.
    virtual const char *getDefaultName() override;

    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[],
                             int n) override;

    virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[],
                           int n) override;

    virtual bool updateProperties() override;

    

protected:
    virtual bool saveConfigItems(FILE *fp) override;

private:
    INDI::PropertyText WhatToSayTP {1};

    INDI::PropertyNumber SayCountNP {1};

    enum
    {
        SAY_HELLO_DEFAULT,
        SAY_HELLO_CUSTOM,
        SAY_HELLO_N,
    };
    INDI::PropertySwitch SayHelloSP {SAY_HELLO_N};    
};
