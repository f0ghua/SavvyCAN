#ifndef JSHELPER_H
#define JSHELPER_H

#include "scriptcontainer.h"

#include <QJSEngine>

class JSHelper : public QObject
{
    Q_OBJECT
public:
    explicit JSHelper(ScriptContainer *sc) : m_scriptContainer(sc) {};

signals:

public slots:
    void log(QString msg);
    bool arrayEqual(QJSValue array1, QJSValue array2, QJSValue length);

private:
    ScriptContainer *m_scriptContainer;
};

#endif // JSHELPER_H
