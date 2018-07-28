#ifndef WIZBUSOCKETHELPER_P_H
#define WIZBUSOCKETHELPER_P_H

#include "wizbusockethelper.h"
#include "wizbusocket.h"

class QTcpSocket;

class WizBuSocketHelperPrivate : public QObject
{
    Q_OBJECT
    Q_DECLARE_PUBLIC(WizBuSocketHelper)
public:
    enum WStatus {
        eIdle = 0,
        eSocketConnected,
        eClientConnected,
        eDeviceConnected
    };

    WizBuSocketHelperPrivate(WizBuSocketHelper *parent);
    virtual ~WizBuSocketHelperPrivate();
    QStringList availablePorts();
    void sendCommand(COMMAND command, const void *pdata, int plen);
    bool isSockConnected() const {return (m_wStatus >= eSocketConnected);}
    void setWStatus(WStatus v) {m_wStatus = v;}
    bool connectToServer();
    void processResponse(COMMAND_DATA_PACKET *commandData);
    void processRequseDevices(QString s);

    WizBuSocketHelper * const q_ptr;
    QTcpSocket *m_socket = NULL;
    WStatus m_wStatus = eIdle;
    QStringList m_devList;
    QByteArray m_receivedData;

public slots:
    void handleReceivedData();

signals:
    void portsChanged();
};

#endif // WIZBUSOCKETHELPER_P_H
