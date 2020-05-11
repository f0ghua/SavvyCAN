#ifndef ISOTP_HANDLER_H
#define ISOTP_HANDLER_H

#include <Qt>
#include <QObject>
#include <QDebug>
#include <QTimer>
#include "can_structs.h"
#include "mainwindow.h"
#include "canframemodel.h"
#include "isotp_message.h"
#include "canfilter.h"

#define ISOTP_CHNNBR    2   // map to each can bus

class ISOTP_HANDLER : public QObject
{
    Q_OBJECT

public:
    ISOTP_HANDLER();
    ~ISOTP_HANDLER();
    void setExtendedAddressing(bool mode);
    void setReception(bool mode); //set whether to accept and forward frames or not
    void sendISOTPFrame(int bus, int ID, QVector<unsigned char> data);
    void setProcessAll(bool state);
    void setFlowCtrl(bool state);
    void addFilter(uint32_t pBusId, uint32_t ID, uint32_t mask);
    void removeFilter(uint32_t pBusId, uint32_t ID, uint32_t mask);
    void clearAllFilters();

public slots:
    void updatedFrames(int);
    void rapidFrames(const CANConnection* conn, const QVector<CANFrame>& pFrames);
    void frameTimerTick_bus1();
    void frameTimerTick_bus2();

signals:
    void newISOMessage(ISOTP_MESSAGE msg);

private:
    QList<ISOTP_MESSAGE> messageBuffer;
    QList<CANFrame> sendingFrames[ISOTP_CHNNBR];
    QList<CANFilter> filters;
    const QVector<CANFrame> *modelFrames;
    bool useExtendedAddressing;
    bool isReceiving;
    bool waitingForFlow[ISOTP_CHNNBR];
    int framesUntilFlow[ISOTP_CHNNBR];
    bool processAll;
    bool issueFlowMsgs;
    QTimer frameTimer[ISOTP_CHNNBR];
    uint32_t lastSenderID;
    uint32_t lastSenderBus;

    void processFrame(const CANFrame &frame);
    void checkNeedFlush(uint64_t ID, int bus);
    void frameTimerTick(int bus);
};

#endif // ISOTP_HANDLER_H
