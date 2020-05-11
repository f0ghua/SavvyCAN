#include "isotp_handler.h"
#include "connections/canconmanager.h"

ISOTP_HANDLER::ISOTP_HANDLER()
{
    useExtendedAddressing = false;
    isReceiving = false;
    issueFlowMsgs = false;
    processAll = false;
    lastSenderBus = 0;
    lastSenderID = 0;

    modelFrames = MainWindow::getReference()->getCANFrameModel()->getListReference();

    connect(&frameTimer[0], SIGNAL(timeout()), this, SLOT(frameTimerTick_bus1()));
    connect(&frameTimer[1], SIGNAL(timeout()), this, SLOT(frameTimerTick_bus2()));
}

ISOTP_HANDLER::~ISOTP_HANDLER()
{
    disconnect(&frameTimer[0], SIGNAL(timeout()), this, SLOT(frameTimerTick_bus1()));
    disconnect(&frameTimer[1], SIGNAL(timeout()), this, SLOT(frameTimerTick_bus2()));
}

void ISOTP_HANDLER::setExtendedAddressing(bool mode)
{
    useExtendedAddressing = mode;
}

void ISOTP_HANDLER::setFlowCtrl(bool state)
{
    issueFlowMsgs = state;
}

void ISOTP_HANDLER::setReception(bool mode)
{
    if (isReceiving == mode) return;
    isReceiving = mode;

    if (isReceiving)
    {
        connect(CANConManager::getInstance(), &CANConManager::framesReceived, this, &ISOTP_HANDLER::rapidFrames);
        qDebug() << "Enabling reception in ISOTP handler";
    }
    else
    {
        disconnect(CANConManager::getInstance(), &CANConManager::framesReceived, this, &ISOTP_HANDLER::rapidFrames);
        qDebug() << "Disabling reception in ISOTP handler";
    }
}

void ISOTP_HANDLER::sendISOTPFrame(int bus, int ID, QVector<unsigned char> data)
{
    CANFrame frame;
    int currByte = 0;
    int index = 1;
    if (bus < 0) return;
    if (bus >= CANConManager::getInstance()->getNumBuses()) return;

    lastSenderID = ID;
    lastSenderBus = bus;

    if (data.length() < 8)
    {
        frame.bus = bus;
        frame.extended = false;
#ifdef VENDOR_SAPA		
		if (ID > 0x7FF) frame.extended = true;
#endif		
        frame.ID = ID;
        frame.len = 8;
        for (int b = 0; b < 8; b++) frame.data[b] = 0x00;
        frame.data[0] = data.length();
        for (int i = 0; i < frame.data[0]; i++) frame.data[i + 1] = data[i];
        CANConManager::getInstance()->sendFrame(frame);
    }
    else //need to send a multi-part ISO_TP message - Respects timing and frame number based flow control
    {
        frame.bus = bus;
        frame.ID = ID;
        frame.extended = false;
#ifdef VENDOR_SAPA		
		if (ID > 0x7FF) frame.extended = true;
#endif		
        frame.len = 8;
        for (int b = 0; b < 8; b++) frame.data[b] = 0x00;
        frame.data[0] = 0x10 + (data.length() / 256);
        frame.data[1] = data.length() & 0xFF;
        for (int i = 0; i < 6; i++) frame.data[2 + i] = data[currByte++];
        CANConManager::getInstance()->sendFrame(frame);
        //Queue up the rest of the frames
        waitingForFlow[bus] = true;
        frameTimer[bus].setInterval(200); //wait a while for the flow frame to come in
        frameTimer[bus].setTimerType(Qt::PreciseTimer);
        frameTimer[bus].start();
        while (currByte < data.length())
        {
            for (int b = 0; b < 8; b++) frame.data[b] = 0x00;
            frame.data[0] = 0x20 + index;
            index = (index + 1) & 0xF;
            int bytesToGo = data.length() - currByte;
            if (bytesToGo > 7) bytesToGo = 7;
            for (int i = 0; i < bytesToGo; i++) frame.data[1 + i] = data[currByte++];
            frame.len = 8;
            sendingFrames[bus].append(frame);
            //CANConManager::getInstance()->sendFrame(frame);
        }
    }
}

//remember, negative numbers are special -1 = all frames deleted, -2 = totally new set of frames.
void ISOTP_HANDLER::updatedFrames(int numFrames)
{
    if (numFrames == -1) //all frames deleted. Kill the display
    {
    }
    else if (numFrames == -2) //all new set of frames. Reset
    {
        for (int i = 0; i < modelFrames->length(); i++) processFrame(modelFrames->at(i));
    }
    else //just got some new frames. See if they are relevant.
    {
        for (int i = modelFrames->count() - numFrames; i < modelFrames->count(); i++)
        {
            //processFrame(modelFrames->at(i));  //accepting these frames in rapidFrames instead
        }
    }
}

void ISOTP_HANDLER::rapidFrames(const CANConnection* conn, const QVector<CANFrame>& pFrames)
{
    Q_UNUSED(conn)
    if (pFrames.length() <= 0) return;

    //qDebug() << "received messages in ISOTP handler";

    foreach(const CANFrame& thisFrame, pFrames)
    {
#ifdef VENDOR_SAPA
		if (!thisFrame.isReceived) continue;
#endif
        //only process frames that we've marked are ISOTP frames
        //unless processAll is true
        if (processAll) processFrame(thisFrame);
        else
        {
            for (int i = 0; i < filters.count(); i++)
            {
                if ((thisFrame.bus == filters[i].bus) && ((thisFrame.ID & filters[i].mask) == filters[i].ID))
                {
                    processFrame(thisFrame);
                    break;
                }
            }

        }
    }
}

void ISOTP_HANDLER::processFrame(const CANFrame &frame)
{
    uint64_t ID = frame.ID;
    int bus = static_cast<int>(frame.bus);
    int frameType;
    int frameLen;
    int ln;
    //int offset;
    ISOTP_MESSAGE msg;
    ISOTP_MESSAGE *pMsg;

    frameType = 0;
    frameLen = 0;

    if (useExtendedAddressing)
    {
        ID = ID << 8;
        ID += frame.data[0];
        frameType = frame.data[1] >> 4;
        frameLen = frame.data[1] & 0xF;
    }
    else
    {
        frameType = frame.data[0] >> 4;
        frameLen = frame.data[0] & 0xF;
    }

    switch(frameType)
    {
    case 0: //single frame message
#ifdef VENDOR_SAPA
        checkNeedFlush(ID, bus);
#else
        checkNeedFlush(ID);
#endif
        if (frameLen == 0) return; //length of zero isn't valid.
        if (frameLen > 6 && useExtendedAddressing) return; //impossible
        if (frameLen > 7) return;

        msg.bus = frame.bus;
        msg.extended = frame.extended;
        msg.ID = ID;
        msg.isReceived = frame.isReceived;
        msg.len = frameLen;
        msg.data.reserve(frameLen);
        msg.timestamp = frame.timestamp;
        if (useExtendedAddressing) for (int j = 0; j < frameLen; j++) msg.data.append(frame.data[j+2]);
        else for (int j = 0; j < frameLen; j++) msg.data.append(frame.data[j+1]);
        //qDebug() << "Emitting single frame ISOTP message";
        emit newISOMessage(msg);
        break;
    case 1: //first frame of a multi-frame message
#ifdef VENDOR_SAPA
        checkNeedFlush(ID, bus);
#else
        checkNeedFlush(ID);
#endif
        msg.bus = frame.bus;
        msg.extended = frame.extended;
        msg.ID = ID;
        msg.timestamp = frame.timestamp;
        msg.isReceived = frame.isReceived;
        frameLen = frameLen << 8;
        if (useExtendedAddressing)
        {
            frameLen += frame.data[2];
            frameLen = frameLen & 0xFFF;
            msg.len = frameLen;
            msg.data.reserve(frameLen);
            for (int j = 0; j < 5; j++) msg.data.append(frame.data[3 + j]);
        }
        else
        {
            frameLen += frame.data[1];
            frameLen = frameLen & 0xFFF;
            msg.len = frameLen;
            msg.data.reserve(frameLen);
            for (int j = 0; j < 6; j++) msg.data.append(frame.data[2 + j]);
        }
        messageBuffer.append(msg);
#ifndef F_NO_DEBUG
        //qDebug() << QObject::tr("receive FF len = %1").arg(msg.len);
#endif
        //The sending ID is set to the last ID we used to send from this class which is
        //very likely to be correct. But, caution, there is a chance that it isn't. Beware.
        if (issueFlowMsgs && lastSenderID > 0)
        {
            CANFrame outFrame;
            outFrame.bus = lastSenderBus;
            outFrame.extended = false;
#ifdef VENDOR_SAPA		
            outFrame.bus = frame.bus;
            if (lastSenderID > 0x7FF) outFrame.extended = true;
#endif

            outFrame.ID = lastSenderID;
            outFrame.len = 8;
            for (int b = 0; b < 8; b++) outFrame.data[b] = 0x00;
            outFrame.data[0] = 0x30; //flow control, go ahead and send
            outFrame.data[1] = 0; //dont ask again about flow control
            outFrame.data[2] = 3; //separation time in milliseconds between messages.
            CANConManager::getInstance()->sendFrame(outFrame);
        }
        break;
    case 2: //subsequent frames for multi-frame messages
        pMsg = NULL;
        for (int i = 0; i < messageBuffer.length(); i++)
        {
            if ((messageBuffer[i].ID == ID)
#ifdef VENDOR_SAPA
                    && (messageBuffer[i].bus == static_cast<int>(frame.bus))
#endif
                    )
            {
                pMsg = &messageBuffer[i];
                break;
            }
        }
        if (!pMsg) return;
        ln = pMsg->len - pMsg->data.count();
#ifndef F_NO_DEBUG
         qDebug() << QObject::tr("bus %1: receive CF left len = %2").arg(pMsg->bus).arg(ln);
#endif

        //offset = pMsg->data.count();
        if (useExtendedAddressing)
        {
            if (ln > 6) ln = 6;
            for (int j = 0; j < ln; j++) pMsg->data.append(frame.data[j+2]);
        }
        else
        {
            if (ln > 7) ln = 7;
            for (int j = 0; j < ln; j++) pMsg->data.append(frame.data[j+1]);
        }
        if (pMsg->len <= pMsg->data.count())
        {
#ifndef F_NO_DEBUG
            qDebug() << "bus" << pMsg->bus << "Emitting multiframe ISOTP message";
#endif
            emit newISOMessage(*pMsg);
        }
        break;
    case 3: //flow control messages
        switch (frameLen) //actually flow control type in this case
        {
        case 0: //continue to send frames but maybe change inter-frame delay
            waitingForFlow[bus] = false;
            //data[1] contains number of frames to send before waiting for next flow control
            framesUntilFlow[bus] = frame.data[1];
            if (framesUntilFlow[bus] == 0) framesUntilFlow[bus] = -1; //-1 means don't count frames and just keep going
            //data[2] contains the interframe delay to use (0xF1 through 0xF9 are special through)
            if (frame.data[2] < 0xF1) frameTimer[bus].start(frame.data[2]); //set proper delay between frames
            else frameTimer[bus].start(1); //can't do sub-millisecond sending with this code so just use 1ms timing
            break;
        case 1: //wait - do not send any more frames until other side says so
            waitingForFlow[bus] = true;
            frameTimer[bus].stop(); //quit sending frames for now
            break;
        case 2: //overflow or abort. Assume this means abort and quit sending
            frameTimer[bus].stop();
            sendingFrames[bus].clear();
            waitingForFlow[bus] = false;
            break;
        }
        waitingForFlow[bus] = false;

        break;
    }
}

void ISOTP_HANDLER::checkNeedFlush(uint64_t ID, int bus)
{
    for (int i = 0; i < messageBuffer.length(); i++)
    {
        if ((messageBuffer[i].ID == ID)
#ifdef VENDOR_SAPA
                && (messageBuffer[i].bus == bus)
#endif
                )
        {
            //used to pass by reference but now newISOMessage should pass by value which makes it easier to use cross thread
            qDebug() << "Flushing a partial frame";
            //emit newISOMessage(messageBuffer[i]); // the msg is wrong, we should not send it out
            messageBuffer.removeAt(i);
            return;
        }
    }
}

void ISOTP_HANDLER::frameTimerTick(int bus)
{
    CANFrame frame;

    if (!waitingForFlow[bus])
    {
        if (!sendingFrames[bus].isEmpty())
        {
            frame = sendingFrames[bus].takeFirst();
            CANConManager::getInstance()->sendFrame(frame);
            if (framesUntilFlow[bus] > -1) framesUntilFlow[bus]--;
            if (framesUntilFlow[bus] == 0) //stop sending and wait for another flow control message
            {
                frameTimer[bus].stop(); //we absolutely will not send anything until other side says to.
                waitingForFlow[bus] = true;
            }
        }
        else //no more frames to send
        {
            frameTimer[bus].stop();
        }
    }
    else //while waiting for a flow frame we didn't get one during timeout period. Try to send anyway with default timeout
    {
        waitingForFlow[bus] = false;
        frameTimer[bus].setInterval(20); //pretty slow sending which should be OK as a default
    }
}

void ISOTP_HANDLER::frameTimerTick_bus1()
{
    frameTimerTick(0);
}

void ISOTP_HANDLER::frameTimerTick_bus2()
{
    frameTimerTick(1);
}

void ISOTP_HANDLER::setProcessAll(bool state)
{
    processAll = state;
}

void ISOTP_HANDLER::addFilter(uint32_t pBusId, uint32_t ID, uint32_t mask)
{
    CANFilter filt;
    filt.ID = ID;
    filt.bus = pBusId;
    filt.mask = mask;

    filters.append(filt);
}

void ISOTP_HANDLER::removeFilter(uint32_t pBusId, uint32_t ID, uint32_t mask)
{
    for (int i = 0; i < filters.count(); i++)
    {
        if (filters[i].bus == pBusId && filters[i].ID == ID && filters[i].mask == mask) filters.removeAt(i);
    }
}

void ISOTP_HANDLER::clearAllFilters()
{
    filters.clear();
}


