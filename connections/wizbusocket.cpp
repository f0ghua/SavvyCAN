#include <QObject>
#include <QDebug>
#include <QCanBusFrame>
#include <QSerialPortInfo>
#include <QSettings>
#include <QStringBuilder>
#include <QHostInfo>
#include <QEventLoop>

#include "wizbusocket.h"
#include "utility.h"

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

bool WizBuSocket::buildCANFrame(CANFrame *frame, const QByteArray &ba)
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
    frame->isReceived = (IS_BIT_SET(c, 1))?false:true; // prevent from ICITS echo

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
    if (m_rxHasCompleteCode) {
        quint8 completeCode = ba.at(i++) & 0xFF;
    }
    if (hasTimeStamp) {
        frame->timestamp = (ba.at(i++) & 0xFF) << 8;
        frame->timestamp |= (ba.at(i) & 0xFF);
    }

    return true;
}

bool WizBuSocket::handleValidateFrames(const QByteArray &cba)
{
    if ((cba.size() == 2) &&
            ((quint8)cba.at(0) == 0x08) &&
            ((quint8)cba.at(1) == 0x92)) {
#ifndef F_NO_DEBUG
        qDebug() << QObject::tr("get version request");
#endif
        QByteArray buffer = QByteArrayLiteral("\x08\x92\x49\x43\x49\x54\x53\x20\x53\x6F\x66\x74\x77\x61\x72\x65\x20\x56\x65\x72\x73\x69\x6F\x6E\x20\x4E\x75\x6D\x62\x65\x72\x3A\x20\x76\x31\x2E\x30\x36\x2E\x32\x37\x2E\x32\xFF\x00");
        SendData(DEVICE_DATA, buffer.data(), buffer.size());
        return true;
    }

    if ((cba.size() == 18) &&
            ((quint8)cba.at(0) == 0x08) &&
            ((quint8)cba.at(1) == 0xE0)) {
#ifndef F_NO_DEBUG
        qDebug() << QObject::tr("get publickey request");
#endif
        QByteArray buffer = QByteArrayLiteral("\x08\xE0\xFF\x00");
        SendData(DEVICE_DATA, buffer.data(), buffer.size());
        return true;
    }

    return false;
}

void WizBuSocket::procRXChar(unsigned char c)
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

bool WizBuSocket::piSendFrame(const CANFrame& frame)
{
    QByteArray buffer;
    quint32 c;
    int ID;

    //qDebug() << "Sending out frame with id " << frame->ID;

    framesRapid++;

	if(!isConnected())
        return false;

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

    if (m_txHasCompleteCode)
    	buffer.append(0x10); // add complete code
    buffer.append(QByteArray::fromRawData(g_frameEndStr, sizeof(g_frameEndStr)));
#ifndef F_NO_DEBUG
    //qDebug() << "writing " << buffer.length() << " bytes to port";
    qDebug() << QObject::tr("send frame[%1]: %2").arg(buffer.count()).arg(buffer.toHex().constData());
#endif
    debugOutput("writing " + QString::number(buffer.length()) + " bytes to port");

    SendData(DEVICE_DATA, buffer.data(), buffer.size());
	return true;
}

void WizBuSocket::SendData(COMMAND command, const void *pdata, int plen)
{
    if (!isConnected())
        return;

	qint32 len = plen + sizeof(DEVICE_DATA_PACKET) + sizeof(COMMAND_DATA_PACKET);
	COMMAND_DATA_PACKET *commandData = (COMMAND_DATA_PACKET *)calloc(len, 1);
	DEVICE_DATA_PACKET *pDeviceData = (DEVICE_DATA_PACKET *) (commandData->Data);

    //qDebug()<<len << sizeof(DEVICE_DATA_PACKET) + sizeof(COMMAND_DATA_PACKET) << m_connId;
    pDeviceData->DeviceID = m_connId;
    //strncpy(pDeviceData->Name, phydev.toLatin1().data(), sizeof(pDeviceData->Name));
	pDeviceData->Size = plen;
	commandData->Command = command;
    commandData->Size = len;

    if(plen != 0 && pdata !=NULL) {
		memcpy(pDeviceData->Data, pdata, plen);
    }

	m_socket->write((const char *)commandData, len);
	
	free(commandData);
}

WizBuSocket::WizBuSocket(QString portName) :
    CANConnection(portName, CANCon::GVRET_SERIAL, 1, 4000, true)
{
    qDebug() << "WizBuSocket()";
    debugOutput("WizBuSocket()");

    m_pTimer = new QTimer(this); /*NB: set this as parent of timer to manage it from working thread */

    rx_step = 0;
    gotValidated = true;
    isAutoRestart = false;

    timeBasis = 0;
    lastSystemTimeBasis = 0;
    timeAtGVRETSync = 0;

    readSettings();
}


WizBuSocket::~WizBuSocket()
{
    stop();
    qDebug() << "~WizBuSocket()";
    debugOutput("~WizBuSocket()");
}


void WizBuSocket::piStarted()
{
    connectDevice();

    /* start timer */
    connect(m_pTimer, SIGNAL(timeout()), this, SLOT(handleTick()));
    m_pTimer->setInterval(250); //tick four times per second
    m_pTimer->setSingleShot(false); //keep ticking
    m_pTimer->start();

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


void WizBuSocket::piSuspend(bool pSuspend)
{
    /* update capSuspended */
    setCapSuspended(pSuspend);

    /* flush queue if we are suspended */
    if(isCapSuspended())
        getQueue().flush();
}


void WizBuSocket::piStop()
{
    m_pTimer->stop();
    disconnectDevice();
}


bool WizBuSocket::piGetBusSettings(int pBusIdx, CANBus& pBus)
{
    return getBusConfig(pBusIdx, pBus);
}


void WizBuSocket::piSetBusSettings(int pBusIdx, CANBus bus)
{
    /* sanity checks */
    if( (pBusIdx < 0) || pBusIdx >= getNumBuses())
        return;

    /* copy bus config */
    setBusConfig(pBusIdx, bus);

    qDebug() << "About to update bus " << pBusIdx << " on WIZBUS";

}


/****************************************************************/
void WizBuSocket::setCompleteCode(bool rx, bool enable)
{
    qDebug() << QObject::tr("setCompleteCode rx = %1, enable = %2").arg(rx).arg(enable);
    if (rx) {
        m_rxHasCompleteCode = enable;
    } else {
        m_txHasCompleteCode = enable;
    }
}

void WizBuSocket::readSettings()
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
        m_txHasCompleteCode = true;
    }
    else m_txHasCompleteCode = false;
*/
}

void WizBuSocket::SendCommand(COMMAND command, const void *pdata, int plen)
{
    if(!isSockConnected() && (command != CLIENT_CONNECT))
        return;

    qint32 len = plen + sizeof(COMMAND_DATA_PACKET);
    COMMAND_DATA_PACKET *commandData = (COMMAND_DATA_PACKET *)calloc(len, 1);

    commandData->Command = command;
    commandData->Size = len;

    if(plen != 0 && pdata != NULL)
        memcpy(commandData->Data, pdata, plen);

#ifndef F_NO_DEBUG
    {
        const char *p = (const char *)commandData;
        QByteArray baData(p, len);
        qDebug() << tr("sendCommand[%1] >> %2").arg(len).arg(baData.toHex(' ').constData());
    }
#endif
    m_socket->write((const char *)commandData, len);
    free(commandData);
}

bool WizBuSocket::connectToServer()
{
    if(m_socket == NULL) {
        m_socket = new QTcpSocket(this);
        connect(m_socket, SIGNAL(readyRead()), this, SLOT(readData()));
        connect(m_socket, SIGNAL(disconnected()), this, SLOT(handleChange()));
    }
    m_socket->connectToHost(m_remoteIp, 8000);
    m_socket->waitForConnected(100);

    if (m_socket->state() != QAbstractSocket::ConnectedState)
        return false;

#ifndef F_NO_DEBUG
    qDebug()<<"CLIENT_CONNECT";
#endif
    QString name = QHostInfo::localHostName();
    SendCommand(CLIENT_CONNECT, name.toLatin1().data(), name.size());
    return true;
}

bool WizBuSocket::lookForServer()
{
    return connectToServer();
}

void WizBuSocket::connectDevice()
{
    QSettings settings;

    if (!lookForServer()) {
        return;
    }

    QString dev = getPort();
    SendCommand(CONNECT_TO_DEVICE, dev.toLatin1().data(), dev.size());

    m_receivedData.clear();
}

void WizBuSocket::disconnectDevice() 
{
    if(m_connId >= 0) {
#ifndef F_NO_DEBUG
        qDebug()<<"DISCONNECT_FROM_DEVICE";
#endif
        SendCommand(DISCONNECT_FROM_DEVICE, &m_connId, sizeof(m_connId));
    }
    
    //updateDeviceConnState(CLOSE_SUCC);

#ifndef F_NO_DEBUG
    qDebug()<<"closeDevice";
#endif

    if (m_socket != NULL) {
        m_socket->close();
        setSockConnected(true);
    }

    setStatus(CANCon::NOT_CONNECTED);
    CANConStatus stats;
    stats.conStatus = getStatus();
    stats.numHardwareBuses = mNumBuses;
    emit status(stats);


    m_connId = -1;
}

QStringList WizBuSocket::availablePorts()
{
#ifndef F_NO_DEBUG
	qDebug()<<"sig REQUEST_DEVICES";
#endif
    if (!isSockConnected()) {
		lookForServer();
	}
	SendCommand(REQUEST_DEVICES, NULL, 0);
    QEventLoop loop;
    QTimer timer;
    
    timer.setSingleShot(true);
    connect(&timer, &QTimer::timeout, &loop, &QEventLoop::quit);
    QObject::connect(this, &WizBuSocket::updateDeviceList, &loop, &QEventLoop::quit);
    timer.start(200);
    loop.exec();

    if (timer.isActive()) {
#ifndef F_NO_DEBUG
        qDebug()<<"get ports successfully";
#endif
    }

    return m_deviceList;
}

void WizBuSocket::readData()
{
    if(!isSockConnected())
        return;

    QByteArray data = m_socket->readAll();
    m_receivedData.append(data);

    COMMAND_DATA_PACKET *pCmdData;
    quint32 len = m_receivedData.size();

#ifndef F_NO_DEBUG
    //qDebug() << QObject::tr("rxData[%1]: d = %2").arg(len).arg(m_receivedData.toHex().constData());
#endif
    while(len > 0) {
        //const char *p = m_receivedData.constData();
        pCmdData = (COMMAND_DATA_PACKET *)(m_receivedData.constData());
        if ((len < sizeof(COMMAND_DATA_PACKET)) ||
                (len < pCmdData->Size)) { //partially data
            break;
        }

        processResponse(pCmdData);
        len -= pCmdData->Size;
        m_receivedData.remove(0, pCmdData->Size);
    }
#ifndef F_NO_DEBUG
    //qDebug() << QObject::tr("sockDataReceived done");
#endif
}

void WizBuSocket::processResponse(COMMAND_DATA_PACKET *commandData)
{
    DEVICE_DATA_PACKET *pDeviceData;
#if 0//ndef F_NO_DEBUG
    qDebug() << QObject::tr("rxReq: c=%1, s=%2, d=%3").\
                arg(commandData->Command).\
                arg(commandData->Size).\
                arg(QByteArray(commandData->Data, commandData->Size-5).toHex().constData());
#endif

    switch(commandData->Command) {

        case CLIENT_CONNECT:
            setSockConnected(true);
            break;

        case REQUEST_VERSION:
            break;

        case DEVICE_DATA:
            pDeviceData = (DEVICE_DATA_PACKET *) (commandData->Data);
            processDeviceData(pDeviceData);
            break;

        case REQUEST_DEVICES:
            processRequseDevices(commandData->Data);
            break;

        case CONNECT_TO_DEVICE:
            m_connId = (qint32)(*(commandData->Data));
#ifndef F_NO_DEBUG
            qDebug()<<"commid " << m_connId;
#endif
            if(m_connId != -1) {
                //emit updateDeviceConnState(OPEN_SUCC);

                setStatus(CANCon::CONNECTED);
                CANConStatus stats;
                stats.conStatus = getStatus();
                stats.numHardwareBuses = mNumBuses;
                emit status(stats);
            }
            else
                SendCommand(CONNECT_TO_DEVICE, phydev.toLatin1().data(), phydev.size());
            break;

        case DEVICE_LIST_CHANGED:
            PhyCloseDevice();
            deleteSocket();
            break;

        default:
#ifndef F_NO_DEBUG
            qDebug()<<"some ev" << commandData->Command;
#endif
            break;
    }
}

void WizBuSocket::processDeviceData(DEVICE_DATA_PACKET *pDeviceData)
{
    QByteArray cba(pDeviceData->Data, pDeviceData->Size);

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
}

void WizBuSocket::processRequseDevices(QString s)
{
    QStringList ql;

    m_deviceList.clear();
    ql = s.split(",");

    for(int i = 0; i < ql.size() - 1; i += 2)
        m_deviceList << ql[i];
#ifndef F_NO_DEBUG
    qDebug()<< "new update" << m_deviceList;
#endif
    emit updateDeviceList(m_deviceList);
}

void WizBuSocket::connectionTimeout()
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

//Debugging data sent from connection window. Inject it into Comm traffic.
void WizBuSocket::debugInput(QByteArray bytes) 
{
   m_socket->write(bytes);
}

void WizBuSocket::rebuildLocalTimeBasis()
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

void WizBuSocket::handleTick()
{

}

