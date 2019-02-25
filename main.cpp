#include "mainwindow.h"
#ifdef VENDOR_SAPA
#include "QAppLogging.h"
#endif
#include <QApplication>

#ifndef F_DISABLE_CRASHDUMP
#include <QMessageBox>
#include "ccrashstack.h"

long __stdcall callback(_EXCEPTION_POINTERS* excp)
{
	CCrashStack crashStack(excp);
	QString sCrashInfo = crashStack.GetExceptionInfo();
	QString sFileName = "busInsightCrash.log";

	QFile file(sFileName);
	if (file.open(QIODevice::WriteOnly|QIODevice::Truncate))
	{
		file.write(sCrashInfo.toUtf8());
		file.close();
	}

	//qDebug()<<"Error:\n"<<sCrashInfo;
	QMessageBox msgBox;
	msgBox.setText(QString::fromUtf8("The application is crashed, please send the log to provider."));
	msgBox.exec();

	return EXCEPTION_EXECUTE_HANDLER;
}
#endif


int main(int argc, char *argv[])
{
#ifndef F_DISABLE_CRASHDUMP
	SetUnhandledExceptionFilter(callback);
#endif
#ifdef VENDOR_SAPA
    QAppLogging::instance()->setLogFileName("scLog.txt");
    QAppLogging::instance()->setOutputDest(QAppLogging::eDestSystem|QAppLogging::eDestFile);
    QAppLogging::installHandler();
    qInfo() << "SavvyCAN is initializing ...";
#endif
    QApplication a(argc, argv);

    MainWindow w;
    w.show();

    return a.exec();
}
