#include <QObject>
#include <QDebug>
#include <QCanBusFrame>
#include <QSerialPortInfo>
#include <QSettings>
#include <QStringBuilder>

#include "wizbuserial.h"

//#ifdef VENDOR_SAPA

#define SAINT_PROTOCOL_ID_MASK  0xF8    // bit 3 ~ 7
#define SAINT_COMMAND_MASK      0x40
#define SAINT_TXRX_MASK         0x20
#define SAINT_TIMESTAMP_MASK    0x01

#define PROTOCOL_ID_CAN1           0x50
#define PROTOCOL_ID_CAN2           0x58
#define PROTOCOL_ID_LIN1           0xB8

#define IS_BIT_SET(data, bit)  (((data)>>(bit))&0x1)
#define SET_BIT(data, bit) (data) = (data)|(1<<(bit))
#define CLEAR_BIT(data, bit) (data) = (data)&(~(1<<(bit)))

static const char g_frameEndStr[2]  = {(char)0xFF, 0x00};
static const quint8 g_frameEscapeChar = 0xFF;

static quint8 lin_calculate_checksum(quint8 *pdata, quint8 len)
{
	quint16 sum = 0, i = 0;
	quint8 checksum = 0;
	quint16 tmp1 = 0 , tmp2 = 0;

	for(i = 0; i < len; i++){
		sum += pdata[i];
		tmp1 = sum >> 8;
		tmp2 = sum & 0xFF;
		sum = tmp1 + tmp2;

	}

	checksum = 0xFF - sum;
	return checksum;
}

bool WizBuSerial::buildCANFrame(CANFrame *frame, const QByteArray &ba)
{
    int protocol;
    int i = 0;
    unsigned char c = ba.at(i++);

    if (ba.count() < 3)
    {
        //qDebug() << "parseByteFrame byteFrame->count() = " << byteFrame->count();
        return false;
    }

    protocol = c & SAINT_PROTOCOL_ID_MASK;
    /* We process CAN frames only */
    if ((protocol != PROTOCOL_ID_CAN1) &&
        (protocol != PROTOCOL_ID_CAN2))
    {
        //qDebug() << "protocol invalid: 0x" << QString::number(protocol, 16).toUpper();
        return false;
    }
    frame->bus = (protocol == PROTOCOL_ID_CAN1)?0:1;
    frame->isReceived = (IS_BIT_SET(c, 1))?false:true;

    bool hasTimeStamp = c & SAINT_TIMESTAMP_MASK;

    c = ba.at(i++);
    frame->extended = (c & 0x80) >> 7;
    if (frame->extended && (ba.count() < 5))
    {
        //qDebug() << "extended frame with size invalid";
        return false;
    }

    if (!frame->extended)
    {
        frame->ID = (c & 0x07) << 8;
        frame->ID |= (ba.at(i++) & 0xFF) ;
    }
    else
    {
        frame->ID = (c & 0x1F) << 24;
        frame->ID |= (ba.at(i++) & 0xFF) << 16;
        frame->ID |= (ba.at(i++) & 0xFF) << 8;
        frame->ID |= (ba.at(i++) & 0xFF);
    }

    frame->len = ba.length() - i;

    // frames received from h/w has 1 byte complete code and 2 bytes timestamp
    // at the tailer. In order to support debug without h/w, we should support
    // both with and w/o these tailer.
    //
    // Since the frames from h/w always has timestamp but from app not, so we do
    // actions on complete code and timestamp when hasTimeStamp flag raised
    // only.

    if (hasTimeStamp)
        frame->len -= (2+1);

    if ((frame->len < 0) || (frame->len > 8))
        return false;

    for (quint32 j = 0; j < frame->len; j++) {
        frame->data[j] = (ba.at(i+j) & 0xFF);
    }

    i += frame->len;
    quint8 completeCode = ba.at(i++) & 0xFF;
    if (hasTimeStamp) {
        frame->timestamp = (ba.at(i++) & 0xFF) << 8;
        frame->timestamp |= (ba.at(i) & 0xFF);
    }

    return true;
}

bool WizBuSerial::handleValidateFrames(const QByteArray &cba)
{
    if ((cba.size() == 2) &&
            ((quint8)cba.at(0) == 0x08) &&
            ((quint8)cba.at(1) == 0x92)) {
#ifndef F_NO_DEBUG
        qDebug() << QObject::tr("get version request");
#endif
        QByteArray buffer = QByteArrayLiteral("\x08\x92\x49\x43\x49\x54\x53\x20\x53\x6F\x66\x74\x77\x61\x72\x65\x20\x56\x65\x72\x73\x69\x6F\x6E\x20\x4E\x75\x6D\x62\x65\x72\x3A\x20\x76\x31\x2E\x30\x36\x2E\x32\x37\x2E\x32\xFF\x00");
        serial->write(buffer);
        return true;
    }

    if ((cba.size() == 18) &&
            ((quint8)cba.at(0) == 0x08) &&
            ((quint8)cba.at(1) == 0xE0)) {
#ifndef F_NO_DEBUG
        qDebug() << QObject::tr("get publickey request");
#endif
        QByteArray buffer = QByteArrayLiteral("\x08\xE0\xFF\x00");
        serial->write(buffer);
        return true;
    }

    return false;
}

void WizBuSerial::procRXChar(unsigned char c)
{
    static unsigned char ffFlag = 0;
    static QByteArray cba = QByteArray();

    if ((c == 0xFF) && (ffFlag == 0)) // first 0xFF
    {
        ffFlag = 1;
        return;
    }

    if (ffFlag == 0)
    {
        cba.append(c);
    }
    else
    {
        ffFlag = 0;
        if (c == 0xFF) // 2 0xFF, the first one means escape character
        {
            //buildFrame->data.append(c);
            cba.append(c);
        }
        else if (c == 0x00) // end of message
        {
            // check if a version request packet, is yes, response it
            if (!handleValidateFrames(cba)) {
                bool isValid = buildCANFrame(&buildFrame, cba);
                if (isValid && (!isCapSuspended()) && buildFrame.isReceived) {
                    /* get frame from queue */
                    CANFrame* frame_p = getQueue().get();
                    if(frame_p) {
                        //qDebug() << "GVRET got frame on bus " << frame_p->bus;
                        /* copy frame */
                        *frame_p = buildFrame;
                        checkTargettedFrame(buildFrame);
                        /* enqueue frame */
                        getQueue().queue();
                    }
                    else
                        qDebug() << "can't get a frame, ERROR";
                }
                    //take the time the frame came in and try to resync the time base.
                    //if (continuousTimeSync) txTimestampBasis = QDateTime::currentMSecsSinceEpoch() - (buildFrame.timestamp / 1000);
            }
            cba.clear();
        }
        else // a new message start
        {
			buildCANFrame(&buildFrame, cba);
			if (!isCapSuspended() && buildFrame.isReceived)
			{
				/* get frame from queue */
				CANFrame* frame_p = getQueue().get();
				if(frame_p) {
					//qDebug() << "GVRET got frame on bus " << frame_p->bus;
					/* copy frame */
					*frame_p = buildFrame;
					checkTargettedFrame(buildFrame);
					/* enqueue frame */
					getQueue().queue();
				}
				else
					qDebug() << "can't get a frame, ERROR";

				//take the time the frame came in and try to resync the time base.
				//if (continuousTimeSync) txTimestampBasis = QDateTime::currentMSecsSinceEpoch() - (buildFrame.timestamp / 1000);
			}
            cba.clear();

            cba.append(c);
        }
    }
}

bool WizBuSerial::piSendFrame(const CANFrame& frame)
{
    QByteArray buffer;
    quint32 c;
    int ID;

    //qDebug() << "Sending out frame with id " << frame->ID;

    framesRapid++;

    if (serial == NULL) return false;
    if (!serial->isOpen()) return false;

    ID = frame.ID;
    if (frame.extended) ID |= 1 << 31;

    if (frame.bus < 2) { // CAN bus
        char protocol = (frame.bus & 1)?PROTOCOL_ID_CAN2:PROTOCOL_ID_CAN1;
        /**
         * bit 1: 1 = Tx
         * bit 0: 0 = No timestamp
         * In order to debug the own App, we alwasy set dir to Rx
         */
        buffer[0] = protocol; //+ (1<<1);
        buffer[1] = 0;
        if (frame.extended) SET_BIT(buffer[1], 7);
        if (!frame.extended) {
            buffer[1] = ((unsigned char)buffer.at(1))|((ID >> 8) & 0x07);
            buffer.append((unsigned char)(ID & 0xFF));
        } else {
            buffer[1] = ((char)buffer[1])|((ID >> 24) & 0x1F);
            buffer.append((unsigned char)(ID >> 16));
            buffer.append((unsigned char)(ID >> 8));
            buffer.append((unsigned char)(ID & 0xFF));
        }

        for (c = 0; c < frame.len; c++) {
            buffer.append(frame.data[c]);
            if ((quint8)frame.data[c] == 0xFF) {
                buffer.append(0xFF);
            }
        }
    } else { // LIN bus
        char protocol = PROTOCOL_ID_LIN1;
        buffer[0] = protocol;
        buffer[1] = ID;

        for (c = 0; c < frame.len; c++) {
            buffer.append(frame.data[c]);
            if ((quint8)frame.data[c] == 0xFF) {
                buffer.append(0xFF);
            }
        }

        const char *data = buffer.constData() + 2;
        quint8 cksum = lin_calculate_checksum((quint8 *)data, buffer.count()-2);
        buffer.append(cksum);
    }

    if (appendCompleteCode)
    	buffer.append(0x10); // add complete code
    buffer.append(QByteArray::fromRawData(g_frameEndStr, sizeof(g_frameEndStr)));
#ifndef F_NO_DEBUG
    //qDebug() << "writing " << buffer.length() << " bytes to serial port";
    qDebug() << QObject::tr("send frame[%1]: %2").arg(buffer.count()).arg(buffer.toHex().constData());
#endif
    debugOutput("writing " + QString::number(buffer.length()) + " bytes to serial port");
    serial->write(buffer);

	return true;
}


WizBuSerial::WizBuSerial(QString portName) :
    CANConnection(portName, CANCon::GVRET_SERIAL, 1, 4000, true),
    mTimer(this) /*NB: set this as parent of timer to manage it from working thread */
{
    qDebug() << "WizBuSerial()";
    debugOutput("WizBuSerial()");

    serial = NULL;
    rx_step = 0;
    gotValidated = true;
    isAutoRestart = false;

    timeBasis = 0;
    lastSystemTimeBasis = 0;
    timeAtGVRETSync = 0;
    appendCompleteCode = false;

    readSettings();
}


WizBuSerial::~WizBuSerial()
{
    stop();
    qDebug() << "~WizBuSerial()";
    debugOutput("~WizBuSerial()");
}


void WizBuSerial::piStarted()
{
    connectDevice();

    /* start timer */
    connect(&mTimer, SIGNAL(timeout()), this, SLOT(handleTick()));
    mTimer.setInterval(250); //tick four times per second
    mTimer.setSingleShot(false); //keep ticking
    mTimer.start();

	can0Baud = 57600;
	mBusData[0].mBus.setSpeed(can0Baud);
	mBusData[0].mBus.setEnabled(can0Enabled);
	mBusData[0].mConfigured = true;
	mBusData[0].mBus.active = true;
	/*
	can1Baud = 57600;
    mBusData[1].mBus.setSpeed(can1Baud);
    mBusData[1].mBus.setEnabled(can1Enabled);
    mBusData[1].mConfigured = true;	
    */
}


void WizBuSerial::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void WizBuSerial::piStop()
{
    mTimer.stop();
    disconnectDevice();
}


bool WizBuSerial::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    return getBusConfig(pBusIdx, pBus);
}


void WizBuSerial::piSetBusSettings(int pBusIdx, CANBus bus)
{
    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    /* copy bus config */
    setBusConfig(pBusIdx, bus);

    qDebug() << "About to update bus " << pBusIdx << " on WIZBUS";

}


/****************************************************************/

void WizBuSerial::readSettings()
{
    QSettings settings;

    if (settings.value("Main/ValidateComm", true).toBool())
    {
        doValidation = true;
    }
    else doValidation = false;

	doValidation=false;
/*
    if (settings.value("Main/CompleteCode", true).toBool())
    {
        appendCompleteCode = true;
    }
    else appendCompleteCode = false;
*/
}


void WizBuSerial::connectDevice()
{
    QSettings settings;

    /* disconnect device */
    if(serial)
        disconnectDevice();

    serial = new QSerialPort(QSerialPortInfo(getPort()));
	if(!serial) {
        qDebug() << "can't open serial port " << getPort();
        debugOutput("can't open serial port " + getPort());
        return;
    }

	serial->setBaudRate(57600);
	serial->setDataBits(QSerialPort::Data8);
	serial->setParity(QSerialPort::NoParity);
	serial->setStopBits(QSerialPort::OneStop);
	serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial->open(QIODevice::ReadWrite))
    {
        qDebug() << serial->errorString();
    }
    //serial->setDataTerminalReady(true); //you do need to set these or the fan gets dirty
    //serial->setRequestToSend(true);


	setStatus(CANCon::CONNECTED);
	CANConStatus stats;
	stats.conStatus = getStatus();
	stats.numHardwareBuses = mNumBuses;
	emit status(stats);

    connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));

}

void WizBuSerial::disconnectDevice() {
    if (serial != NULL)
    {
        if (serial->isOpen())
        {
            serial->clear();
            serial->close();

        }
        serial->disconnect(); //disconnect all signals
        delete serial;
        serial = NULL;
    }
}


void WizBuSerial::connectionTimeout()
{
    //one second after trying to connect are we actually connected?
    //if (CANCon::NOT_CONNECTED==getStatus()) //no?
    if (!gotValidated)
    {
        //then emit the the failure signal and see if anyone cares
        qDebug() << "Failed to connect to GVRET at that com port";

        disconnectDevice();
    }
}


void WizBuSerial::readSerialData()
{
    QByteArray data = serial->readAll();
    unsigned char c;
    QString debugBuild;
    debugOutput("Got data from serial. Len = " % QString::number(data.length()));
    //qDebug() << (tr("Got data from serial. Len = %0").arg(data.length()));
    for (int i = 0; i < data.length(); i++)
    {
        c = data.at(i);
        //qDebug() << c << "    " << QString::number(c, 16) << "     " << QString(c);
        debugBuild = debugBuild % QString::number(c, 16) % " ";
        procRXChar(c);
    }
    debugOutput(debugBuild);
}

//Debugging data sent from connection window. Inject it into Comm traffic.
void WizBuSerial::debugInput(QByteArray bytes) {
   serial->write(bytes);
}




void WizBuSerial::rebuildLocalTimeBasis()
{
    qDebug() << "Rebuilding GVRET time base. GVRET local base = " << buildTimeBasis;

    /*
      our time basis is the value we have to modulate the main system basis by in order
      to sync the GVRET timestamps to the rest of the system.
      The rest of the system uses CANConManager::getInstance()->getTimeBasis as the basis.
      GVRET returns to us the current time since boot up in microseconds.
      timeAtGVRETSync stores the "system" timestamp when the GVRET timestamp was retrieved.
    */
    lastSystemTimeBasis = CANConManager::getInstance()->getTimeBasis();
    int64_t systemDelta = timeAtGVRETSync - lastSystemTimeBasis;
    int32_t localDelta = buildTimeBasis - systemDelta;
    timeBasis = -localDelta;
}

void WizBuSerial::handleTick()
{
    if (lastSystemTimeBasis != CANConManager::getInstance()->getTimeBasis()) rebuildLocalTimeBasis();
    //qDebug() << "Tick!";
/*
    if( CANCon::CONNECTED == getStatus() )
    {
        if (!gotValidated && doValidation)
        {
            if (serial == NULL) return;
            if (serial->isOpen()) //if it's still false we have a problem...
            {
                qDebug() << "Comm validation failed. ";

                setStatus(CANCon::NOT_CONNECTED);
                emit status(getStatus());

                disconnectDevice(); //start by stopping everything.
                //Then wait 500ms and restart the connection automatically
                QTimer::singleShot(500, this, SLOT(connectDevice()));
                return;
            }
        }
    }
*/
    if (doValidation && serial && serial->isOpen()) sendCommValidation();
}


void WizBuSerial::sendCommValidation()
{
    QByteArray output;

    gotValidated = false;
    output.append((char)0xF1); //another command to the GVRET
    output.append((char)0x09); //request a reply to get validation
    //send it twice for good measure.
    output.append((char)0xF1); //another command to the GVRET
    output.append((char)0x09); //request a reply to get validation

    serial->write(output);
}
