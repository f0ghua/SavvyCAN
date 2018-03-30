#include <QJSValueIterator>
#include <QDebug>

#include "scriptcontainer.h"
#include "connections/canconmanager.h"

#ifdef VENDOR_SAPA
#include "jshelper.h"

void JSHelper::log(QString msg) 
{
	qDebug() << "jsHelper: "<< msg;
	emit m_scriptContainer->sendLog(msg);
}

bool JSHelper::arrayEqual(QJSValue array1, QJSValue array2, QJSValue length)
{		
	if ((!array1.isArray())||(!array2.isArray())) 
		qDebug() << "data isn't an array";

	uint32_t len = length.toUInt();

    for (int i = 0; i < len; i++) {
        uint8_t v1 = static_cast<uint8_t>(array1.property(i).toInt());
        uint8_t v2 = static_cast<uint8_t>(array2.property(i).toInt());
        if (v1 != v2 ){
			return false;
		}
	}

	return true;
}
#endif

ScriptContainer::ScriptContainer()
{
    canHelper = new CANScriptHelper(&scriptEngine);
    isoHelper = new ISOTPScriptHelper(&scriptEngine);
    udsHelper = new UDSScriptHelper(&scriptEngine);
    connect(&timer, SIGNAL(timeout()), this, SLOT(tick()));

#ifdef VENDOR_SAPA

	//Add a bunch of helper objects into javascript that the scripts
	//can use to interact with the CAN buses
	QJSValue hostObj = scriptEngine.newQObject(this);
	scriptEngine.globalObject().setProperty("host", hostObj);
	QJSValue canObj = scriptEngine.newQObject(canHelper);
	scriptEngine.globalObject().setProperty("can", canObj);
	QJSValue isoObj = scriptEngine.newQObject(isoHelper);
	scriptEngine.globalObject().setProperty("isotp", isoObj);
	QJSValue udsObj = scriptEngine.newQObject(udsHelper);
	scriptEngine.globalObject().setProperty("uds", udsObj);

	JSHelper *jsHelper = new JSHelper(this);
	QJSValue jshObj = scriptEngine.newQObject(jsHelper);
	scriptEngine.globalObject().setProperty("jsh", jshObj);

#endif
}

void ScriptContainer::compileScript()
{
    QJSValue result = scriptEngine.evaluate(scriptText, fileName);

#ifndef VENDOR_SAPA
	emit sendLog("Compiling script...");
#else
    emit sendLog("Evaluate script done.");
#endif

    canHelper->clearFilters();
    isoHelper->clearFilters();
    udsHelper->clearFilters();

    if (result.isError())
    {

        emit sendLog("SCRIPT EXCEPTION!");
        emit sendLog("Line: " + result.property("lineNumber").toString());
        emit sendLog(result.property("message").toString());
        emit sendLog("Stack:");
        emit sendLog(result.property("stack").toString());
    }
    else
    {
#ifndef VENDOR_SAPA    
        compiledScript = result;

        //Add a bunch of helper objects into javascript that the scripts
        //can use to interact with the CAN buses
        QJSValue hostObj = scriptEngine.newQObject(this);
        scriptEngine.globalObject().setProperty("host", hostObj);
        QJSValue canObj = scriptEngine.newQObject(canHelper);
        scriptEngine.globalObject().setProperty("can", canObj);
        QJSValue isoObj = scriptEngine.newQObject(isoHelper);
        scriptEngine.globalObject().setProperty("isotp", isoObj);
        QJSValue udsObj = scriptEngine.newQObject(udsHelper);
        scriptEngine.globalObject().setProperty("uds", udsObj);
#endif

        //Find out which callbacks the script has created.
        setupFunction = scriptEngine.globalObject().property("setup");
        canHelper->setRxCallback(scriptEngine.globalObject().property("gotCANFrame"));
        isoHelper->setRxCallback(scriptEngine.globalObject().property("gotISOTPMessage"));
        udsHelper->setRxCallback(scriptEngine.globalObject().property("gotUDSMessage"));

        tickFunction = scriptEngine.globalObject().property("tick");

        if (setupFunction.isCallable())
        {
            qDebug() << "setup exists";
            QJSValue res = setupFunction.call();
            if (res.isError())
            {
                emit sendLog("Error in setup function on line " + res.property("lineNumber").toString());
                emit sendLog(res.property("message").toString());
            }
        }
        if (tickFunction.isCallable()) qDebug() << "tick exists";
		
    }
}

void ScriptContainer::setScriptWindow(ScriptingWindow *win)
{
    window = win;
    connect(this, &ScriptContainer::sendLog, window, &ScriptingWindow::log);
}

void ScriptContainer::log(QJSValue logString)
{
    QString val = logString.toString();
    emit sendLog(val);
}

void ScriptContainer::setTickInterval(QJSValue interval)
{
    int intervalValue = interval.toInt();
    qDebug() << "called set tick interval with value " << intervalValue;
    if (intervalValue > 0)
    {
        timer.setInterval(intervalValue);
        timer.start();
    }
    else timer.stop();
}

void ScriptContainer::tick()
{
    if (tickFunction.isCallable())
    {
        //qDebug() << "Calling tick function";
        QJSValue res = tickFunction.call();
        if (res.isError())
        {
            emit sendLog("Error in tick function on line " + res.property("lineNumber").toString());
            emit sendLog(res.property("message").toString());
        }
    }
}

void ScriptContainer::addParameter(QJSValue name)
{
    scriptParams.append(name.toString());
}

void ScriptContainer::updateValuesTable(QTableWidget *widget)
{
    QString valu;

    foreach (QString paramName, scriptParams)
    {
        valu = scriptEngine.globalObject().property(paramName).toString();
        qDebug() << paramName << " - " << valu;
        bool found = false;
        for (int i = 0; i < widget->rowCount(); i++)
        {
            if (widget->item(i, 0) && widget->item(i, 0)->text().compare(paramName) == 0)
            {
                found = true;
                if (!widget->item(i, 1)->isSelected())
                {
                    widget->item(i,1)->setText(valu);
                }
                break;
            }
        }
        if (!found)
        {
            int row = widget->rowCount();
            widget->insertRow(widget->rowCount());
            QTableWidgetItem *item;
            item = new QTableWidgetItem();
            item->setText(paramName);
            item->setFlags(Qt::ItemIsEnabled);
            widget->setItem(row, 0, item);
            item = new QTableWidgetItem();
            item->setText(valu);
            widget->setItem(row, 1, item);
        }
    }
}

void ScriptContainer::updateParameter(QString name, QString value)
{
    qDebug() << name << " * " << value;
    QJSValue val(value);
    scriptEngine.globalObject().setProperty(name, val);
}




/* CANScriptHandler Methods */

CANScriptHelper::CANScriptHelper(QJSEngine *engine)
{
    scriptEngine = engine;
}

void CANScriptHelper::setRxCallback(QJSValue cb)
{
    gotFrameFunction = cb;
}

void CANScriptHelper::setFilter(QJSValue id, QJSValue mask, QJSValue bus)
{
    uint32_t idVal = id.toUInt();
    uint32_t maskVal = mask.toUInt();
    int busVal = bus.toInt();
    qDebug() << "Called set filter";
    qDebug() << idVal << "*" << maskVal << "*" << busVal;
    CANFilter filter;
    filter.setFilter(idVal, maskVal, busVal);
    filters.append(filter);

    CANConManager::getInstance()->addTargettedFrame(busVal, idVal, maskVal, this);
}

void CANScriptHelper::clearFilters()
{
    qDebug() << "Called clear filters";
    foreach (CANFilter filter, filters)
    {
        CANConManager::getInstance()->removeTargettedFrame(filter.bus, filter.ID, filter.mask, this);
    }

    filters.clear();
}

void CANScriptHelper::sendFrame(QJSValue bus, QJSValue id, QJSValue length, QJSValue data)
{
    CANFrame frame;
    frame.extended = false;
    frame.ID = id.toInt();
    frame.len = length.toUInt();
    if (frame.len > 8) frame.len = 8;

    if (!data.isArray()) qDebug() << "data isn't an array";

    for (unsigned int i = 0; i < frame.len; i++)
    {
        frame.data[i] = (uint8_t)data.property(i).toInt();
    }

    frame.bus = (uint32_t)bus.toInt();
    //if (frame.bus > 1) frame.bus = 1;

    if (frame.ID > 0x7FF) frame.extended = true;

    qDebug() << "sending frame from script";
    CANConManager::getInstance()->sendFrame(frame);
}

void CANScriptHelper::gotTargettedFrame(const CANFrame &frame)
{
    if (!gotFrameFunction.isCallable()) return; //nothing to do if we can't even call the function
    //qDebug() << "Got frame in script interface";
    for (int i = 0; i < filters.length(); i++)
    {
        if (filters[i].checkFilter(frame.ID, frame.bus))
        {
            QJSValueList args;
            args << frame.bus << frame.ID << frame.len;
            QJSValue dataBytes = scriptEngine->newArray(frame.len);

            for (unsigned int j = 0; j < frame.len; j++) dataBytes.setProperty(j, QJSValue(frame.data[j]));
            args.append(dataBytes);
            gotFrameFunction.call(args);
            return; //as soon as one filter matches we jump out
        }
    }
}




/* ISOTPScriptHelper methods */
ISOTPScriptHelper::ISOTPScriptHelper(QJSEngine *engine)
{
    scriptEngine = engine;
    handler = new ISOTP_HANDLER;
    connect(handler, SIGNAL(newISOMessage(ISOTP_MESSAGE)), this, SLOT(newISOMessage(ISOTP_MESSAGE)));
    handler->setReception(true);
}

void ISOTPScriptHelper::clearFilters()
{
    handler->clearAllFilters();
}

void ISOTPScriptHelper::setFilter(QJSValue id, QJSValue mask, QJSValue bus)
{
    uint32_t idVal = id.toUInt();
    uint32_t maskVal = mask.toUInt();
    int busVal = bus.toInt();
    qDebug() << "Called isotp set filter";
    qDebug() << idVal << "*" << maskVal << "*" << busVal;

    handler->addFilter(busVal, idVal, maskVal);
}

void ISOTPScriptHelper::sendISOTP(QJSValue bus, QJSValue id, QJSValue length, QJSValue data)
{
    ISOTP_MESSAGE msg;
    msg.extended = false;
    msg.ID = id.toUInt();
    msg.len = length.toUInt();

    if (!data.isArray()) qDebug() << "data isn't an array";
#ifdef VENDOR_SAPA
    msg.data.resize(msg.len);
#endif
    for (int i = 0; i < msg.len; i++)
    {
        msg.data[i] = static_cast<uint8_t>(data.property(i).toInt());
    }

    msg.bus = bus.toInt();

    if (msg.ID > 0x7FF) msg.extended = true;

    qDebug() << "sending isotp message from script";
    handler->sendISOTPFrame(msg.bus, msg.ID, msg.data);
}

#ifdef VENDOR_SAPA
void ISOTPScriptHelper::setFCOn()
{
    handler->setFlowCtrl(true);
	handler->setProcessAll(true);
}

void ISOTPScriptHelper::setFlowCtrl(QJSValue state)
{
	bool en = state.toBool();
    handler->setFlowCtrl(en);
}

void ISOTPScriptHelper::setProcessAll(QJSValue state)
{
	bool en = state.toBool();
	handler->setProcessAll(en);
}

void ISOTPScriptHelper::send15765(QJSValue bus, QJSValue id, QJSValue length, QJSValue data)
{
    ISOTP_MESSAGE msg;
    msg.extended = false;
    msg.ID = id.toInt();
    msg.len = length.toUInt();

	QString dataStr = data.toString();
    QByteArray ba = QByteArray::fromHex(dataStr.toLatin1());    

    msg.data.reserve(msg.len);
    for (int i = 0; i < msg.len; i++)
    {
        msg.data.append(ba.at(i));
    }

    msg.bus = bus.toInt();

    if (msg.ID > 0x7FF) msg.extended = true;

    qDebug() << "sending isotp message from script";
    handler->sendISOTPFrame(msg.bus, msg.ID, msg.data);
}
#endif

void ISOTPScriptHelper::setRxCallback(QJSValue cb)
{
    gotFrameFunction = cb;
}

void ISOTPScriptHelper::newISOMessage(ISOTP_MESSAGE msg)
{
    qDebug() << "isotpScriptHelper got a ISOTP message";
    if (!gotFrameFunction.isCallable()) return; //nothing to do if we can't even call the function
    qDebug() << "Got frame in script interface";

    QJSValueList args;
    args << msg.bus << msg.ID << msg.len;
    QJSValue dataBytes = scriptEngine->newArray(static_cast<uint>(msg.len));

    for (unsigned int j = 0; j < msg.len; j++) dataBytes.setProperty(j, QJSValue(msg.data[j]));
    args.append(dataBytes);
    gotFrameFunction.call(args);
}




/* UDSScriptHelper methods */
UDSScriptHelper::UDSScriptHelper(QJSEngine *engine)
{
    scriptEngine = engine;
    handler = new UDS_HANDLER;
    connect(handler, SIGNAL(newUDSMessage(UDS_MESSAGE)), this, SLOT(newUDSMessage(UDS_MESSAGE)));
    handler->setReception(true);
}

void UDSScriptHelper::clearFilters()
{
    handler->clearAllFilters();
}

void UDSScriptHelper::setFilter(QJSValue id, QJSValue mask, QJSValue bus)
{
    uint32_t idVal = id.toUInt();
    uint32_t maskVal = mask.toUInt();
    int busVal = bus.toInt();
    qDebug() << "Called uds set filter";
    qDebug() << idVal << "*" << maskVal << "*" << busVal;

    handler->addFilter(busVal, idVal, maskVal);
}

void UDSScriptHelper::sendUDS(QJSValue bus, QJSValue id, QJSValue service, QJSValue sublen, QJSValue subFunc, QJSValue length, QJSValue data)
{
    UDS_MESSAGE msg;
    msg.extended = false;
    msg.ID = id.toUInt();
    msg.len = length.toUInt();
    msg.service = service.toUInt();
    msg.subFuncLen = sublen.toUInt();
    msg.subFunc = subFunc.toUInt();

    if (!data.isArray()) qDebug() << "data isn't an array";

    msg.data.reserve(msg.len);

    for (unsigned int i = 0; i < msg.len; i++)
    {
        msg.data.append(static_cast<uint8_t>(data.property(i).toInt()));
    }

    msg.bus = (uint32_t)bus.toInt();

    if (msg.ID > 0x7FF) msg.extended = true;

    qDebug() << "sending UDS message from script";

    handler->sendUDSFrame(msg);
}

void UDSScriptHelper::setRxCallback(QJSValue cb)
{
    gotFrameFunction = cb;
}

void UDSScriptHelper::newUDSMessage(UDS_MESSAGE msg)
{
    //qDebug() << "udsScriptHelper got a UDS message";
    qDebug() << "UDS script helper. Meg data len: " << msg.len;
    if (!gotFrameFunction.isCallable()) return; //nothing to do if we can't even call the function
    //qDebug() << "Got frame in script interface";

    QJSValueList args;
    args << msg.bus << msg.ID << msg.service << msg.subFunc << msg.len;
    QJSValue dataBytes = scriptEngine->newArray(msg.len);

    for (unsigned int j = 0; j < msg.data.length(); j++) dataBytes.setProperty(j, QJSValue(msg.data[j]));
    args.append(dataBytes);
    gotFrameFunction.call(args);
}

