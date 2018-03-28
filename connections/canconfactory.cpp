#include <QString>
#include "canconfactory.h"
#include "serialbusconnection.h"
#include "gvretserial.h"
#include "wizbuserial.h"

using namespace CANCon;

CANConnection* CanConFactory::create(type pType, QString pPortName)
{
    switch(pType) {
        case SOCKETCAN:
            return new SerialBusConnection(pPortName);
        case GVRET_SERIAL:
#ifdef VENDOR_SAPA			
            return new WizBuSerial(pPortName);	
#else			
            return new GVRetSerial(pPortName);
#endif

        default: {}
    }

    return NULL;
}
