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
#include "QAppLogging.h"

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

static void packFrame(QByteArray &data)
{
	int pos, from = 0;

	// escape 0xFF
	while ((pos = data.indexOf(g_frameEscapeChar, from)) != -1) {
		data.insert(pos, g_frameEscapeChar);
		from = pos + 2; //jump 2 escape chars
	}

	// append the end
	data.append(QByteArray::fromRawData(g_frameEndStr, sizeof(g_frameEndStr)));
}

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
        sendData(DEVICE_DATA, buffer.data(), buffer.size());
        return true;
    }

    if ((cba.size() == 18) &&
            ((quint8)cba.at(0) == 0x08) &&
            ((quint8)cba.at(1) == 0xE0)) {
#ifndef F_NO_DEBUG
        qDebug() << QObject::tr("get publickey request");
#endif
        QByteArray buffer = QByteArrayLiteral("\x08\xE0\xFF\x00");
        sendData(DEVICE_DATA, buffer.data(), buffer.size());
        return true;
    }

    return false;
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

    sendData(DEVICE_DATA, buffer.data(), buffer.size());
	return true;
}

void WizBuSocket::sendData(COMMAND command, const void *pdata, int plen)
{
    if (!isConnected()) {
        return;
    }

	qint32 len = plen + sizeof(DEVICE_DATA_PACKET) + sizeof(COMMAND_DATA_PACKET);
	COMMAND_DATA_PACKET *commandData = (COMMAND_DATA_PACKET *)calloc(len, 1);
    DEVICE_DATA_PACKET *pDeviceData = (DEVICE_DATA_PACKET *) (commandData->data);

    //qDebug()<<len << sizeof(DEVICE_DATA_PACKET) + sizeof(COMMAND_DATA_PACKET) << m_connId;
    pDeviceData->deviceID = m_connId;
    //strncpy(pDeviceData->Name, phydev.toLatin1().data(), sizeof(pDeviceData->Name));
    pDeviceData->size = plen;
    commandData->command = command;
    commandData->size = len;

    if(plen != 0 && pdata !=NULL) {
        memcpy(pDeviceData->data, pdata, plen);
    }

	m_socket->write((const char *)commandData, len);

#ifndef F_NO_DEBUG
{
    QByteArray ba((char *)commandData, len);
    qDebug() << QObject::tr("sendData [%1]: %2").arg(len).arg(ba.toHex(' ').constData());
}
#endif    
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

        /* connect device */
    if (!connectDevice()) {
        disconnectDevice();
        qDebug() << "can't connect device";
    }

    qDebug() << "About to update bus " << pBusIdx << " on WIZBUS";

    return;
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

void WizBuSocket::setLocalDevConnected(bool isDevConnected)
{
#ifndef F_NO_DEBUG
    qDebug() << QObject::tr("setLocalDevConnected %1").arg(isDevConnected);
#endif
    if (isDevConnected) {
        m_isLocalDevConnected = true;
        m_rxHasCompleteCode = true;
    } else {
        m_isLocalDevConnected = false;
        m_rxHasCompleteCode = false;
    }
}

void WizBuSocket::setRemoteDevConnected(bool isDevConnected)
{
#ifndef F_NO_DEBUG
    qDebug() << QObject::tr("setRemoteDevConnected %1").arg(isDevConnected);
#endif
    if (isDevConnected) {
        m_isRemoteDevConnected = true;
        m_txHasCompleteCode = true;
    } else {
        m_isRemoteDevConnected = false;
        m_txHasCompleteCode = false;
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
    Qt::CheckState isLocalDevConnected = static_cast<Qt::CheckState>(settings.value("icitsconn/isLocalDevConnected").value<int>());
    setLocalDevConnected((isLocalDevConnected == Qt::Checked)?true:false);

    Qt::CheckState isRemoteDevConnected = static_cast<Qt::CheckState>(settings.value("icitsconn/isRemoteDevConnected").value<int>());
    setRemoteDevConnected((isRemoteDevConnected == Qt::Checked)?true:false);

}

void WizBuSocket::sendCommand(COMMAND command, const void *pdata, int plen)
{
    if(!isSockConnected() && (command != CLIENT_CONNECT))
        return;

    qint32 len = plen + sizeof(COMMAND_DATA_PACKET);
    COMMAND_DATA_PACKET *commandData = (COMMAND_DATA_PACKET *)calloc(len, 1);

    commandData->command = command;
    commandData->size = len;

    if(plen != 0 && pdata != NULL)
        memcpy(commandData->data, pdata, plen);

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
        //connect(m_socket, SIGNAL(disconnected()), this, SLOT(handleChange()));
    }
    m_socket->connectToHost(QHostAddress::LocalHost, 8000);
    m_socket->waitForConnected(100);

    if (m_socket->state() != QAbstractSocket::ConnectedState)
        return false;

#ifndef F_NO_DEBUG
    qDebug() << "CLIENT_CONNECT";
#endif
    setWStatus(eSocketConnected);
    QString name = QHostInfo::localHostName();
    sendCommand(CLIENT_CONNECT, name.toLatin1().data(), name.size());
    return true;
}

bool WizBuSocket::lookForServer()
{
    return connectToServer();
}

bool WizBuSocket::connectDevice()
{
    QSettings settings;

    if (!lookForServer()) {
        return false;
    }

    QString dev = getPort();
    sendCommand(CONNECT_TO_DEVICE, dev.toLatin1().data(), dev.size());

    m_receivedData.clear();
    return true;
}

void WizBuSocket::disconnectDevice() 
{
    if(m_connId >= 0) {
#ifndef F_NO_DEBUG
        qDebug()<<"DISCONNECT_FROM_DEVICE";
#endif
        sendCommand(DISCONNECT_FROM_DEVICE, &m_connId, sizeof(m_connId));
    }
    
    //updateDeviceConnState(CLOSE_SUCC);

#ifndef F_NO_DEBUG
    qDebug()<<"closeDevice";
#endif

    if (m_socket != NULL) {
        m_socket->close();
    }

    setWStatus(eIdle);

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
    sendCommand(REQUEST_DEVICES, NULL, 0);

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
                (len < pCmdData->size)) { //partially data
            break;
        }

        processResponse(pCmdData);
        len -= pCmdData->size;
        m_receivedData.remove(0, pCmdData->size);
    }
#ifndef F_NO_DEBUG
    //qDebug() << QObject::tr("sockDataReceived done");
#endif
}

void WizBuSocket::processResponse(COMMAND_DATA_PACKET *commandData)
{
    DEVICE_DATA_PACKET *pDeviceData;
#ifndef F_NO_DEBUG
    qDebug() << QObject::tr("rxReq: c=%1, s=%2, d=%3").\
                arg(commandData->command).\
                arg(commandData->size).\
                arg(QByteArray(commandData->data, commandData->size-5).toHex(' ').constData());
#endif

    switch(commandData->command) {

        case CLIENT_CONNECT:
            setWStatus(eClientConnected);
            break;

        case REQUEST_VERSION:
            break;

        case DEVICE_DATA:
            pDeviceData = (DEVICE_DATA_PACKET *) (commandData->data);
            processDeviceData(pDeviceData);
            break;

        case REQUEST_DEVICES:
            processRequseDevices(commandData->data);
            break;

        case CONNECT_TO_DEVICE:
            m_connId = (qint32)(*(commandData->data));
#ifndef F_NO_DEBUG
            qDebug()<<"commid " << m_connId;
#endif
            if(m_connId != -1) {
                //emit updateDeviceConnState(OPEN_SUCC);

                setWStatus(eDeviceConnected);

                setStatus(CANCon::CONNECTED);
                CANConStatus stats;
                stats.conStatus = getStatus();
                stats.numHardwareBuses = mNumBuses;
                emit status(stats);
            }
            else {
                //sendCommand(CONNECT_TO_DEVICE, phydev.toLatin1().data(), phydev.size());
            }
            break;

        case DEVICE_LIST_CHANGED:
            disconnectDevice();
            break;

        default:
#ifndef F_NO_DEBUG
            qDebug()<<"some ev" << commandData->command;
#endif
            break;
    }
}

void WizBuSocket::processDeviceData(DEVICE_DATA_PACKET *pDeviceData)
{
    if (isCapSuspended())
        return;
    
    QByteArray cba(pDeviceData->data, pDeviceData->size);

    if (!m_isLocalDevConnected) {
        if (handleValidateFrames(cba))
            return;

        buildFrame.isReceived = true;
    }

    bool isValid = buildCANFrame(&buildFrame, cba);
    if (!isValid)
        return;

    if (!m_isLocalDevConnected) {
        buildFrame.isReceived = true;
    } else if (!buildFrame.isReceived) {
        return;
    }
    
    /* get frame from queue */
    CANFrame* frame_p = getQueue().get();
    if(frame_p) {
        /* copy frame */
        *frame_p = buildFrame;
        checkTargettedFrame(buildFrame);
        /* enqueue frame */
        getQueue().queue();
    }
    else {
        qDebug() << "can't get a frame, ERROR";
    }

    if (!m_isRemoteDevConnected) {
        cba.append(0x10); // add complete code
        packFrame(cba);
        sendData(DEVICE_DATA, cba.constData(), cba.size());
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

