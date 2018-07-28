#ifndef WIZBUSOCKET_H
#define WIZBUSOCKET_H

#include <QCanBusDevice>
#include <QThread>
#include <QTimer>
#include <QTcpSocket>

/*************/
#include <QDateTime>
/*************/

#include "canframemodel.h"
#include "canconnection.h"
#include "canconmanager.h"

typedef enum
{
    CLIENT_CONNECT = 0,
    CLIENT_DISCONNECT = 1,
    CLIENT_KEEPALIVE = 2,
    REQUEST_DEVICES = 3,
    CONNECT_TO_DEVICE = 4,
    DISCONNECT_FROM_DEVICE = 5,
    DEVICE_DATA = 6,
    REQUEST_VERSION = 7,
    DEVICE_LIST_CHANGED = 8,
    DEVICE_RAW_DATA = 9,
    OPEN_ALL_DEVICES = 10,
    CLOSE_ALL_DEVICES = 11,
    OPEN_DEVICE_BY_NAME = 12,
    CLOSE_DEVICE_BY_NAME = 13,
    OPEN_DEVICE_BY_ID = 14,
    CLOSE_DEVICE_BY_ID = 15,
    AM_I_FIRST = 16,
    SET_FILTER = 17,
    CLOSE_FILTER = 18,
    REQUEST_FILTERS = 19,
    GLOBAL_ENABLE_FILTERS = 20,
    FTDI_DATA = 21,
    FTDI_DATA_R = 22,
    CREATE_VS = 23,
    DESTROY_VS = 24,
    UPDATE_TIMESTAMP = 25
} COMMAND;

typedef struct
{
    qint32 size;//4
    qint8 command; //1
    char data[0];
}__attribute__((packed)) COMMAND_DATA_PACKET; //Size = 8, Data[1] + 2 free bytes = 3 free bytes

typedef struct
{
    qint32 deviceID; //4
    qint32 size; //4
    char data[0]; //1
}__attribute__((packed)) DEVICE_DATA_PACKET;

class WizBuSocket : public CANConnection
{
    Q_OBJECT

public:
    enum WStatus {
        eIdle = 0,
        eSocketConnected,
        eClientConnected,
        eDeviceConnected
    };

    WizBuSocket(QString portName);
    virtual ~WizBuSocket();
    bool rxCompleteCode() const {return m_rxHasCompleteCode;}
    bool txCompleteCode() const {return m_txHasCompleteCode;}
	bool localDevConnected() const {return m_isLocalDevConnected;}
	bool remoteDevConnected() const {return m_isRemoteDevConnected;}
    QStringList availablePorts();
	
protected:

    virtual void piStarted();
    virtual void piStop();
    virtual void piSetBusSettings(int pBusIdx, CANBus pBus);
    virtual bool piGetBusSettings(int pBusIdx, CANBus& pBus);
    virtual void piSuspend(bool pSuspend);
    virtual bool piSendFrame(const CANFrame&) ;

signals:
    void requestPorts();
	void updateDeviceList(QStringList portList);
	
public slots:
    void debugInput(QByteArray bytes);
	void setCompleteCode(bool rx, bool enable);
	void setLocalDevConnected(bool isConnected);
	void setRemoteDevConnected(bool isConnected);
	
private slots:
    void connectDevice();
    void connectionTimeout();
    void handleTick();
    void readData();

private:
    void readSettings();
    //void sendCommValidation();
    void rebuildLocalTimeBasis();
	bool buildCANFrame(CANFrame *frame, const QByteArray &ba);
    bool handleValidateFrames(const QByteArray &cba);
    bool connectToServer();
    void disconnectDevice();
    bool lookForServer();
    bool isConnected() {return getStatus() == CANCon::CONNECTED;}
    bool isSockConnected() const {return (m_wStatus >= eSocketConnected);}
    void setWStatus(WStatus v) {m_wStatus = v;}
    void sendCommand(COMMAND command, const void *pdata, int plen);
    void sendData(COMMAND command, const void *pdata, int plen);
    void processDeviceData(DEVICE_DATA_PACKET *pDeviceData);
    void processResponse(COMMAND_DATA_PACKET *commandData);
    void processRequseDevices(QString s);

protected:
    QTimer *m_pTimer;
    QThread mThread;

    QTcpSocket *m_socket = NULL;
    WStatus m_wStatus = eIdle;
    bool m_isSockConnected = false;
	QStringList m_deviceList;
    qint32 m_connId = -1;
    QByteArray m_receivedData;
    bool doValidation;
    bool gotValidated;
    bool isAutoRestart;
    bool continuousTimeSync;
    int framesRapid;
    uint32_t rx_step;
    CANFrame buildFrame;
    int can0Baud, can1Baud, swcanBaud, lin1Baud, lin2Baud;
    bool can0Enabled, can1Enabled, swcanEnabled, lin1Enabled, lin2Enabled;
    bool can0ListenOnly, can1ListenOnly, swcanListenOnly;
    int deviceBuildNum;
    int deviceSingleWireMode;
    uint32_t buildTimeBasis;
    int32_t timeBasis;
    uint64_t lastSystemTimeBasis;
    uint64_t timeAtGVRETSync;
    bool m_txHasCompleteCode = false;
	bool m_rxHasCompleteCode = true;
	bool m_isLocalDevConnected = false;
	bool m_isRemoteDevConnected = false;
};

#endif

