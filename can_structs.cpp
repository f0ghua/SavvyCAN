#include "can_structs.h"

#ifdef VENDOR_SAPA	
QList<QByteArray> CommandFrame::sstDataList;

CommandFrame::CommandFrame(QByteArray &raw)
{
    int i = 0;
    isValidFrame = false;

    if (raw.isEmpty())
        return;

    quint8 header = raw.at(i++);
    protocol = header & SAINT_PROTOCOL_ID_MASK;
    switch (protocol) {
        case PROTOCOL_ID_CAN1:
            bus = 0;
            break;
        case PROTOCOL_ID_CAN2:
            bus = 1;
            break;
        case PROTOCOL_ID_LIN1:
            bus = 2;
            break;
        case PROTOCOL_ID_CONFIGURATION:
            break;
        default:
            return;
    }
    isProtocolCommand = IS_BIT_SET(header, 2);

    if (!isProtocolCommand
        && (protocol != PROTOCOL_ID_CONFIGURATION)) {
        return;
    }

    if (raw.size() < 2) {
        return;
    }
    action = raw.at(i++);
    data = raw.mid(i);

    responseDataList.clear();
    processCommandData();

    isValidFrame = true;
}

void CommandFrame::processLinCommands()
{
    switch (action) {
        case 0x10:
        {
            sstDataList.append(data);
            break;
        }
        case 0x13:
        {
            quint8 idx = data.at(0);
            if (idx == 0xFF) {
                for (int i = 0; i < sstDataList.count(); i++) {
                    QByteArray ba = QByteArrayLiteral("\xBC\x13\x00\x01");
                    ba.append(sstDataList.at(i));
                    responseDataList.append(ba);
                }
            }

            break;
        }
        default:
            break;
    }

}

void CommandFrame::processCommandData()
{
    if (isProtocolCommand) {
        switch (protocol) {
            case PROTOCOL_ID_CAN1:
            case PROTOCOL_ID_CAN2:
                break;
            case PROTOCOL_ID_LIN1:
                processLinCommands();
                break;
            default:
                break;
        }
    }
}

bool CANFrame::buildFrame(QByteArray &ba)
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
    bus = (protocol == PROTOCOL_ID_CAN1)?0:1;
    isReceived = (IS_BIT_SET(c, 1))?false:true;

    bool hasTimeStamp = c & SAINT_TIMESTAMP_MASK;

    c = ba.at(i++);
    extended = (c & 0x80) >> 7;
    if (extended && (ba.count() < 5))
    {
        //qDebug() << "extended frame with size invalid";
        return false;
    }

    if (!extended)
    {
        ID = (c & 0x07) << 8;
        ID |= (ba.at(i++) & 0xFF) ;
    }
    else
    {
        ID = (c & 0x1F) << 24;
        ID |= (ba.at(i++) & 0xFF) << 16;
        ID |= (ba.at(i++) & 0xFF) << 8;
        ID |= (ba.at(i++) & 0xFF);
    }

    len = ba.length() - i;

	// frames received from h/w has 1 byte complete code and 2 bytes timestamp
	// at the tailer. In order to support debug without h/w, we should support
	// both with and w/o these tailer.
	//
	// Since the frames from h/w always has timestamp but from app not, so we do
	// actions on complete code and timestamp when hasTimeStamp flag raised
	// only.

    if (hasTimeStamp)
        len -= (2+1);

    if ((len < 0) || (len > 8))
    	return false;

    for (int j = 0; j < len; j++) {
    	data[j] = (ba.at(i+j) & 0xFF);
    }

    i += len;
	quint8 completeCode = ba.at(i++) & 0xFF;
    if (hasTimeStamp) {
        timestamp = (ba.at(i++) & 0xFF) << 8;
        timestamp |= (ba.at(i) & 0xFF);
    }

	return true;
}
#endif

