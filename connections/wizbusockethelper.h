#ifndef WIZBUSOCKETHELPER_H
#define WIZBUSOCKETHELPER_H

#include <QObject>

class WizBuSocketHelperPrivate;

class WizBuSocketHelper : public QObject
{
    Q_OBJECT
    Q_DECLARE_PRIVATE(WizBuSocketHelper)
public:
    explicit WizBuSocketHelper(QObject *parent = nullptr);
    ~WizBuSocketHelper();
    QStringList availablePorts() const;

private:
    WizBuSocketHelperPrivate * const d_ptr;
};

#endif // WIZBUSOCKETHELPER_H
