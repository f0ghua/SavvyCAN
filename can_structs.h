#ifndef CAN_STRUCTS_H
#define CAN_STRUCTS_H

#include <QVector>
#include <stdint.h>

#ifdef VENDOR_SAPA
#include <QByteArray.h>

#define SAINT_PROTOCOL_ID_MASK  0xF8    // bit 3 ~ 7
#define SAINT_COMMAND_MASK      0x40
#define SAINT_TXRX_MASK         0x20
#define SAINT_TIMESTAMP_MASK    0x01

#define PROTOCOL_ID_CAN1           0x50
#define PROTOCOL_ID_CAN2           0x58

#define IS_BIT_SET(data, bit)  (((data)>>(bit))&0x1)
#define SET_BIT(data, bit) (data) = (data)|(1<<(bit))
#define CLEAR_BIT(data, bit) (data) = (data)&(~(1<<(bit)))
#endif

class CANFrame
{
public:
#ifdef VENDOR_SAPA	
	bool buildFrame(QByteArray &ba);
#endif
    int ID;
    int bus;
    bool extended;
    bool isReceived; //did we receive this or send it?
    int len;
    unsigned char data[8];
    uint64_t timestamp;
};

struct J1939ID
{
public:
    int src;
    int dest;
    int pgn;
    int pf;
    int ps;
    int priority;
    bool isBroadcast;
};

//the same as the CANFrame struct but with arbitrary data size.
struct ISOTP_MESSAGE
{
public:
    int ID;
    int bus;
    bool extended;
    bool isReceived;
    int len; //# of bytes this message should have (as reported)
    int actualSize; //# we actually got
    QVector<unsigned char> data;
    uint64_t timestamp;
};

#endif // CAN_STRUCTS_H

