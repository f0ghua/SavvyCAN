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
    void frameTimerTick();

signals:
    void newISOMessage(ISOTP_MESSAGE msg);

private:
    QList<ISOTP_MESSAGE> messageBuffer;
    QList<CANFrame> sendingFrames;
    QList<CANFilter> filters;
    const QVector<CANFrame> *modelFrames;
    bool useExtendedAddressing;
    bool isReceiving;
    bool waitingForFlow;
    int framesUntilFlow;
    bool processAll;
    bool issueFlowMsgs;
    QTimer frameTimer;
    uint32_t lastSenderID;
    uint32_t lastSenderBus;

    void processFrame(const CANFrame &frame);
    void checkNeedFlush(uint64_t ID, int bus);
};

#endif // ISOTP_HANDLER_H
