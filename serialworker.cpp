#include "serialworker.h"

#include <QSerialPort>
#include <QDebug>
#include <QTimer>
#include <QSettings>

SerialWorker::SerialWorker(CANFrameModel *model, QObject *parent) : QObject(parent)
{
    serial = NULL;
    rx_state = IDLE;
    rx_step = 0;
    buildFrame = new CANFrame;
    canModel = model;
    ticker = NULL;
    framesRapid = 0;
    capturing = true;
    gotValidated = true;
    isAutoRestart = false;
    targetID = -1;

    txTimestampBasis = QDateTime::currentMSecsSinceEpoch();

    readSettings();
}

SerialWorker::~SerialWorker()
{
    if (serial != NULL)
    {
        if (serial->isOpen())
        {
            serial->clear();
            serial->close();

        }
        serial->disconnect(); //disconnect all signals
        delete serial;
    }
    if (ticker != NULL) ticker->stop();
}

void SerialWorker::run()
{
    ticker = new QTimer;
    connect(ticker, SIGNAL(timeout()), this, SLOT(handleTick()));

    ticker->setInterval(250); //tick four times per second
    ticker->setSingleShot(false); //keep ticking
    ticker->start();
}

void SerialWorker::readSettings()
{
    QSettings settings;

    if (settings.value("Main/ValidateComm", true).toBool())
    {
        doValidation = true;
    }
    else doValidation = false;

#ifdef VENDOR_SAPA
	doValidation=false;

    if (settings.value("Main/CompleteCode", true).toBool())
    {
        appendCompleteCode = true;
    }
    else appendCompleteCode = false;	
#endif
}

#ifdef VENDOR_SAPA
void SerialWorker::setSerialPort(QSerialPortInfo *port)
{
    QSettings settings;

    currentPort = port;

    if (serial != NULL)
    {
        if (serial->isOpen())
        {
            serial->clear();
            serial->close();
        }
        serial->disconnect(); //disconnect all signals
        delete serial;
    }

    serial = new QSerialPort(*port);

    qDebug() << "Serial port name is " << port->portName();

	serial->setBaudRate(57600);
	serial->setDataBits(QSerialPort::Data8);
	serial->setParity(QSerialPort::NoParity);
	serial->setStopBits(QSerialPort::OneStop);
	serial->setFlowControl(QSerialPort::NoFlowControl);

    if (!serial->open(QIODevice::ReadWrite))
    {
        qDebug() << serial->errorString();
    }

    connected = true;
	emit connectionSuccess(can0Baud, can1Baud);
    connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));


}
#else
void SerialWorker::setSerialPort(QSerialPortInfo *port)
{
    QSettings settings;

    currentPort = port;

    if (serial != NULL)
    {
        if (serial->isOpen())
        {
            serial->clear();
            serial->close();
        }
        serial->disconnect(); //disconnect all signals
        delete serial;
    }

    serial = new QSerialPort(*port);

    qDebug() << "Serial port name is " << port->portName();
    //serial->setBaudRate(10000000); //more speed! probably does nothing for USB serial
    serial->setDataBits(serial->Data8);
    serial->setFlowControl(serial->HardwareControl); //this is important though
    if (!serial->open(QIODevice::ReadWrite))
    {
        qDebug() << serial->errorString();
    }
    serial->setDataTerminalReady(true); //you do need to set these or the fan gets dirty
    serial->setRequestToSend(true);
    QByteArray output;
    output.append((char)0xE7); //this puts the device into binary comm mode
    output.append((char)0xE7);

    output.append((char)0xF1); //signal we want to issue a command
    output.append((char)0x06); //request canbus stats from the board

    output.append((char)0xF1); //another command to the GVRET
    output.append((char)0x07); //request device information

    output.append((char)0xF1);
    output.append((char)0x08); //setting singlewire mode
    if (settings.value("Main/SingleWireMode", false).toBool())
    {
        output.append((char)0x10); //signal that we do want single wire mode
    }
    else
    {
        output.append((char)0xFF); //signal we don't want single wire mode
    }

    output.append((char)0xF1); //yet another command
    output.append((char)0x09); //comm validation command

    output.append((char)0xF1); //and another command
    output.append((char)0x01); //Time Sync - Not implemented until 333 but we can try

    continuousTimeSync = true;

    serial->write(output);
    if (doValidation) connected = false;
        else connected = true;
    connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));
    if (doValidation) QTimer::singleShot(1000, this, SLOT(connectionTimeout()));
}
#endif

void SerialWorker::connectionTimeout()
{
    //one second after trying to connect are we actually connected?
    if (!connected) //no?
    {
        //then emit the the failure signal and see if anyone cares
        qDebug() << "Failed to connect to GVRET at that com port";
        //ticker->stop();
        closeSerialPort(); //make sure it's properly closed anyway
        emit connectionFailure();
    }
}

void SerialWorker::readSerialData()
{
    QByteArray data = serial->readAll();
    unsigned char c;
    //qDebug() << (tr("Got data from serial. Len = %0").arg(data.length()));
    for (int i = 0; i < data.length(); i++)
    {
        c = data.at(i);
        procRXChar(c);
    }
    if (framesRapid > 0)
    {
        emit frameUpdateRapid(framesRapid);
        framesRapid = 0;
    }
}

#ifdef VENDOR_SAPA

static const char g_frameEndStr[2]  = {(char)0xFF, 0x00};
static const quint8 g_frameEscapeChar = 0xFF;

void SerialWorker::procRXChar(unsigned char c)
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
            /*
            buildFrame->buildFrame(cba);
            if (capturing)
            {
                buildFrame->isReceived = true;
                canModel->addFrame(*buildFrame, false);
                    //take the time the frame came in and try to resync the time base.
                if (continuousTimeSync) txTimestampBasis = QDateTime::currentMSecsSinceEpoch() - (buildFrame->timestamp / 1000);
                framesRapid++;
                if (buildFrame->ID == targetID) emit gotTargettedFrame(canModel->rowCount() - 1);
            }
            */
            handleCompleteFrame(cba);
            cba.clear();
        }
        else // a new message start
        {
            /*
            buildFrame->buildFrame(cba);
            if (capturing)
            {
                buildFrame->isReceived = true;
                canModel->addFrame(*buildFrame, false);
                    //take the time the frame came in and try to resync the time base.
                if (continuousTimeSync) txTimestampBasis = QDateTime::currentMSecsSinceEpoch() - (buildFrame->timestamp / 1000);
                framesRapid++;
                if (buildFrame->ID == targetID) emit gotTargettedFrame(canModel->rowCount() - 1);
            }
            */
            handleCompleteFrame(cba);
            cba.clear();

            cba.append(c);
        }
    }
}

void SerialWorker::handleCompleteFrame(QByteArray &raw)
{
    CommandFrame cf(raw);
    if (cf.isValid()) {
        QList<QByteArray> baList;

        baList = cf.getResponse();
        if (!baList.isEmpty()) {
            for (int i = 0; i < baList.count(); i++) {
                const QByteArray &ba = baList.at(i);
                sendRawData(ba);
            }
        }
        return;
    }

    buildFrame->buildFrame(raw);
    if (capturing)
    {
        buildFrame->isReceived = true;
        canModel->addFrame(*buildFrame, false);
            //take the time the frame came in and try to resync the time base.
        if (continuousTimeSync) txTimestampBasis = QDateTime::currentMSecsSinceEpoch() - (buildFrame->timestamp / 1000);
        framesRapid++;
        if (buildFrame->ID == targetID) emit gotTargettedFrame(canModel->rowCount() - 1);
    }
}

static QByteArray packFrame(const QByteArray &data)
{
    QByteArray buffer;

    // escape 0xFF
    for (int i = 0; i < data.count(); i++) {
        buffer.append(data.at(i));
        if (data.at(i) == g_frameEscapeChar) {
            buffer.append(g_frameEscapeChar);
        }
    }

    // append the end
    buffer.append(QByteArray::fromRawData(g_frameEndStr, sizeof(g_frameEndStr)));

    return buffer;
}

void SerialWorker::sendRawData(const QByteArray &raw)
{
    if (serial == NULL) return;
    if (!serial->isOpen()) return;
    if (!connected) return;

    QByteArray buffer = packFrame(raw);

#ifndef F_NO_DEBUG
    //qDebug() << "writing " << buffer.length() << " bytes to serial port";
    qDebug() << QObject::tr("send frame[%1]: %2").arg(buffer.count()).arg(buffer.toHex().constData());
#endif

    serial->write(buffer);
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

void SerialWorker::sendFrame(const CANFrame *frame, int bus = 0)
{
    QByteArray buffer;
    int c;
    int ID;
    CANFrame tempFrame = *frame;
    tempFrame.isReceived = false;
    tempFrame.timestamp = ((QDateTime::currentMSecsSinceEpoch() - txTimestampBasis) * 1000);

    //qDebug() << "Sending out frame with id " << frame->ID;

    //show our sent frames in the list too. This happens even if we're not connected.
    canModel->addFrame(tempFrame, false);
    framesRapid++;

    if (serial == NULL) return;
    if (!serial->isOpen()) return;
    if (!connected) return;

    ID = frame->ID;
    if (frame->extended) ID |= 1 << 31;

    if (bus < 2) { // CAN bus
        char protocol = (bus & 1)?PROTOCOL_ID_CAN2:PROTOCOL_ID_CAN1;
        /**
         * bit 1: 1 = Tx
         * bit 0: 0 = No timestamp
         * In order to debug the own App, we alwasy set dir to Rx
         */
        buffer[0] = protocol; //+ (1<<1);
        buffer[1] = 0;
        if (frame->extended) SET_BIT(buffer[1], 7);
        if (!frame->extended) {
            buffer[1] = ((unsigned char)buffer.at(1))|((ID >> 8) & 0x07);
            buffer.append((unsigned char)(ID & 0xFF));
        } else {
            buffer[1] = ((char)buffer[1])|((ID >> 24) & 0x1F);
            buffer.append((unsigned char)(ID >> 16));
            buffer.append((unsigned char)(ID >> 8));
            buffer.append((unsigned char)(ID & 0xFF));
        }

        for (c = 0; c < frame->len; c++) {
            buffer.append(frame->data[c]);
            if ((quint8)frame->data[c] == 0xFF) {
                buffer.append(0xFF);
            }
        }
    } else { // LIN bus
        char protocol = PROTOCOL_ID_LIN1;
        buffer[0] = protocol;
        buffer[1] = ID;
        
        for (c = 0; c < frame->len; c++) {
            buffer.append(frame->data[c]);
            if ((quint8)frame->data[c] == 0xFF) {
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

    //qDebug() << "writing " << buffer.length() << " bytes to serial port";
    serial->write(buffer);
}
#else
void SerialWorker::sendFrame(const CANFrame *frame, int bus = 0)
{
    QByteArray buffer;
    int c;
    int ID;
    CANFrame tempFrame = *frame;
    tempFrame.isReceived = false;
    tempFrame.timestamp = ((QDateTime::currentMSecsSinceEpoch() - txTimestampBasis) * 1000);

    //qDebug() << "Sending out frame with id " << frame->ID;

    //show our sent frames in the list too. This happens even if we're not connected.
    canModel->addFrame(tempFrame, false);
    framesRapid++;

    if (serial == NULL) return;
    if (!serial->isOpen()) return;
    if (!connected) return;

    ID = frame->ID;
    if (frame->extended) ID |= 1 << 31;

    buffer[0] = (char)0xF1; //start of a command over serial
    buffer[1] = 0; //command ID for sending a CANBUS frame
    buffer[2] = (unsigned char)(ID & 0xFF); //four bytes of ID LSB first
    buffer[3] = (unsigned char)(ID >> 8);
    buffer[4] = (unsigned char)(ID >> 16);
    buffer[5] = (unsigned char)(ID >> 24);
    buffer[6] = (unsigned char)(bus & 1);
    buffer[7] = (unsigned char)frame->len;
    for (c = 0; c < frame->len; c++)
    {
        buffer[8 + c] = frame->data[c];
    }
    buffer[8 + frame->len] = 0;

    //qDebug() << "writing " << buffer.length() << " bytes to serial port";
    serial->write(buffer);
}
#endif

//a simple way for another thread to pass us a bunch of frames to send.
//Don't get carried away here. The GVRET firmware only has finite
//buffers and besides, the other end will get buried in traffic.
void SerialWorker::sendFrameBatch(const QList<CANFrame> *frames)
{
    sendBulkMutex.lock();
    for (int i = 0; i < frames->length(); i++) sendFrame(&frames->at(i), frames->at(i).bus);
    sendBulkMutex.unlock();
}

void SerialWorker::updateBaudRates(int Speed1, int Speed2)
{
    QByteArray buffer;
    qDebug() << "Got signal to update bauds. 1: " << Speed1 <<" 2: " << Speed2;
    buffer[0] = (char)0xF1; //start of a command over serial
    buffer[1] = 5; //setup canbus
    buffer[2] = (unsigned char)(Speed1 & 0xFF); //four bytes of ID LSB first
    buffer[3] = (unsigned char)(Speed1 >> 8);
    buffer[4] = (unsigned char)(Speed1 >> 16);
    buffer[5] = (unsigned char)(Speed1 >> 24);
    buffer[6] = (unsigned char)(Speed2 & 0xFF); //four bytes of ID LSB first
    buffer[7] = (unsigned char)(Speed2 >> 8);
    buffer[8] = (unsigned char)(Speed2 >> 16);
    buffer[9] = (unsigned char)(Speed2 >> 24);
    buffer[10] = 0;
    if (serial == NULL) return;
    if (!serial->isOpen()) return;
    serial->write(buffer);
}

#ifdef VENDOR_SAPA
void SerialWorker::updateCanSettings(bool completeCode)
{
	appendCompleteCode = completeCode;
}
#endif

#ifndef VENDOR_SAPA
void SerialWorker::procRXChar(unsigned char c)
{
    switch (rx_state)
    {
    case IDLE:
        if (c == 0xF1) rx_state = GET_COMMAND;
        break;
    case GET_COMMAND:
        switch (c)
        {
        case 0: //receiving a can frame
            rx_state = BUILD_CAN_FRAME;
            rx_step = 0;
            break;
        case 1: //time sync
            rx_state = TIME_SYNC;
            rx_step = 0;
            break;
        case 2: //process a return reply for digital input states.
            rx_state = GET_DIG_INPUTS;
            rx_step = 0;
            break;
        case 3: //process a return reply for analog inputs
            rx_state = GET_ANALOG_INPUTS;
            break;
        case 4: //we set digital outputs we don't accept replies so nothing here.
            rx_state = IDLE;
            break;
        case 5: //we set canbus specs we don't accept replies.
            rx_state = IDLE;
            break;
        case 6: //get canbus parameters from GVRET
            rx_state = GET_CANBUS_PARAMS;
            rx_step = 0;
            break;
        case 7: //get device info
            rx_state = GET_DEVICE_INFO;
            rx_step = 0;
            break;
        case 9:
            gotValidated = true;
            //qDebug() << "Got validated";
            rx_state = IDLE;
            break;
        }
        break;
    case BUILD_CAN_FRAME:
        switch (rx_step)
        {
        case 0:
            buildFrame->timestamp = c;
            break;
        case 1:
            buildFrame->timestamp |= (uint)(c << 8);
            break;
        case 2:
            buildFrame->timestamp |= (uint)c << 16;
            break;
        case 3:
            buildFrame->timestamp |= (uint)c << 24;
            break;
        case 4:
            buildFrame->ID = c;
            break;
        case 5:
            buildFrame->ID |= c << 8;
            break;
        case 6:
            buildFrame->ID |= c << 16;
            break;
        case 7:
            buildFrame->ID |= c << 24;
            if ((buildFrame->ID & 1 << 31) == 1 << 31)
            {
                buildFrame->ID &= 0x7FFFFFFF;
                buildFrame->extended = true;
            }
            else buildFrame->extended = false;
            break;
        case 8:
            buildFrame->len = c & 0xF;
            if (buildFrame->len > 8) buildFrame->len = 8;
            buildFrame->bus = (c & 0xF0) >> 4;
            break;
        default:
            if (rx_step < buildFrame->len + 9)
            {
                buildFrame->data[rx_step - 9] = c;
            }
            else
            {
                rx_state = IDLE;
                rx_step = 0;
                //qDebug() << "emit from serial handler to main form id: " << buildFrame->ID;
                if (capturing)
                {
                    buildFrame->isReceived = true;
                    canModel->addFrame(*buildFrame, false);
                    //take the time the frame came in and try to resync the time base.
                    if (continuousTimeSync) txTimestampBasis = QDateTime::currentMSecsSinceEpoch() - (buildFrame->timestamp / 1000);
                    framesRapid++;
                    if (buildFrame->ID == targetID) emit gotTargettedFrame(canModel->rowCount() - 1);
                }
            }
            break;
        }
        rx_step++;
        break;
    case TIME_SYNC: //gives a pretty good base guess for the proper timestamp. Can be refined when traffic starts to flow (if wanted)
        switch (rx_step)
        {
        case 0:
            buildTimeBasis = c;
            break;
        case 1:
            buildTimeBasis += ((uint32_t)c << 8);
            break;
        case 2:
            buildTimeBasis += ((uint32_t)c << 16);
            break;
        case 3:
            buildTimeBasis += ((uint32_t)c << 24);
            qDebug() << "GVRET firmware reports timestamp of " << buildTimeBasis;
            txTimestampBasis = QDateTime::currentMSecsSinceEpoch() - ((uint64_t)buildTimeBasis / (uint64_t)1000ull);
            continuousTimeSync = false;
            rx_state = IDLE;
            break;
        }
        rx_step++;
        break;

    case GET_ANALOG_INPUTS: //get 9 bytes - 2 per analog input plus checksum
        switch (rx_step)
        {
        case 0:
            break;
        }
        rx_step++;
        break;
    case GET_DIG_INPUTS: //get two bytes. One for digital in status and one for checksum.
        switch (rx_step)
        {
        case 0:
            break;
        case 1:
            rx_state = IDLE;
            break;
        }
        rx_step++;
        break;
    case GET_CANBUS_PARAMS:
        switch (rx_step)
        {
        case 0:
            can0Enabled = c;
            break;
        case 1:
            can0Baud = c;
            break;
        case 2:
            can0Baud |= c << 8;
            break;
        case 3:
            can0Baud |= c << 16;
            break;
        case 4:
            can0Baud |= c << 24;
            break;
        case 5:
            can1Enabled = c;
            break;
        case 6:
            can1Baud = c;
            break;
        case 7:
            can1Baud |= c << 8;
            break;
        case 8:
            can1Baud |= c << 16;
            break;
        case 9:
            can1Baud |= c << 24;
            rx_state = IDLE;
            qDebug() << "Baud 0 = " << can0Baud;
            qDebug() << "Baud 1 = " << can1Baud;
            if (!can1Enabled) can1Baud = 0;
            if (!can0Enabled) can0Baud = 0;
            connected = true;
            emit connectionSuccess(can0Baud, can1Baud);
            break;
        }
        rx_step++;
        break;
    case GET_DEVICE_INFO:
        switch (rx_step)
        {
        case 0:
            deviceBuildNum = c;
            break;
        case 1:
            deviceBuildNum |= c << 8;
            break;
        case 2:
            break; //don't care about eeprom version
        case 3:
            break; //don't care about file type
        case 4:
            break; //don't care about whether it auto logs or not
        case 5:
            deviceSingleWireMode = c;
            rx_state = IDLE;
            qDebug() << "build num: " << deviceBuildNum;
            qDebug() << "single wire can: " << deviceSingleWireMode;
            emit deviceInfo(deviceBuildNum, deviceSingleWireMode);
            break;
        }
        rx_step++;
        break;
    case SET_DIG_OUTPUTS:
        rx_state = IDLE;
        break;
    case SETUP_CANBUS:
        rx_state = IDLE;
        break;
    case SET_SINGLEWIRE_MODE:
        rx_state = IDLE;
        break;

    }
}
#endif

void SerialWorker::handleTick()
{
    //qDebug() << "Tick!";

    if (connected)
    {
        if (!gotValidated && doValidation)
        {
            if (serial == NULL) return;
            if (serial->isOpen()) //if it's still false we have a problem...
            {
                qDebug() << "Comm validation failed. ";
                closeSerialPort(); //start by stopping everything.
                //Then wait 500ms and restart the connection automatically
                QTimer::singleShot(500, this, SLOT(handleReconnect()));
                return;
            }
        }
    }

    if (doValidation && serial && serial->isOpen()) sendCommValidation();
}

void SerialWorker::handleReconnect()
{
    qDebug() << "Automatically reopening the connection";
    setSerialPort(currentPort); //then go back through the re-init
}

void SerialWorker::sendCommValidation()
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

//totally shuts down the whole thing
void SerialWorker::closeSerialPort()
{
    if (serial == NULL) return;
    if (serial->isOpen())
    {
        serial->clear();
        serial->close();
    }
    serial->disconnect();
    //do not stop the ticker here. It always stays running now.
    //ticker->stop();
    delete serial;
    serial = NULL;
}

void SerialWorker::stopFrameCapture()
{
    qDebug() << "Stopping frame capture";
    capturing = false;
}

void SerialWorker::startFrameCapture()
{
    qDebug() << "Starting up frame capture";
    capturing = true;
}

void SerialWorker::targetFrameID(int target)
{
    targetID = target;
}
