#include "QAppLogging.h"
#include "windows.h"

#include <QFile>
#include <QDir>
#include <QDateTime>
#include <QMutex>
#include <QCoreApplication>

#define LOG_INTKEY "appCore"

QAPP_LOGGING_CATEGORY(AppCore,            LOG_INTKEY)
QAPP_LOGGING_CATEGORY(AppCoreTrace,       LOG_INTKEY QAL_TAG_TRACE)

QAtomicPointer<QAppLogging> QAppLogging::s_instance = 0;

static QtMessageHandler g_oldMsgHandle;

static void msgHandler(QtMsgType type,
                    const QMessageLogContext &context,
                    const QString &message)
{
    static QMutex mutex;
    QMutexLocker lock(&mutex);

    QAppLogging *appLogging = QAppLogging::instance();
    QString logMessage;
    do {
        int destOption = appLogging->outputDest();
        if (destOption == QAppLogging::eDestNone) {
            break;
        }

        logMessage = qFormatLogMessage(type, context, message);
        logMessage.append(QLatin1Char('\n'));

        if (destOption & QAppLogging::eDestSystem) {
            OutputDebugString(reinterpret_cast<const wchar_t *>(logMessage.utf16()));
        }

        if (destOption & QAppLogging::eDestFile) {
            QFile *logFile = appLogging->logFile();
            if (logFile) {
                QTextStream stream(logFile);
                stream << qPrintable(logMessage);
                stream.flush();
            }
        }
    } while(0);

    switch (type) {
    case QtFatalMsg:
        abort();
        break;
    case QtWarningMsg:
//        if (logMessage.contains("Cannot create children")) {
//            asm("nop");
//        }
        break;
    default:
        break;
    }
}

QAppLogging::QAppLogging()
    : m_outputDest(eDestSystem)
    , m_logFileDir()
    , m_logFileName()
    , m_maxFileSize(1024*1024*256)
    , m_logFile(NULL)
{

}

void QAppLogging::installHandler()
{
    g_oldMsgHandle = qInstallMessageHandler(msgHandler);
    qSetMessagePattern("[%{time yyyyMMdd h:mm:ss.zzz} %{if-debug}D%{endif}%{if-info}I%{endif}%{if-warning}W%{endif}%{if-critical}C%{endif}%{if-fatal}F%{endif}] %{file}:%{line} - %{message}");
}

bool QAppLogging::createLogFile()
{
    bool ret = false;

    QDateTime dtmCur = QDateTime::currentDateTime();
    QCoreApplication * app = QCoreApplication::instance();
    QString logDir = m_logFileDir;
    QString logFileName = m_logFileName;
    if (logDir.isEmpty()) {
        logDir = app->applicationDirPath() + "/log/" + dtmCur.toString("yyyy_MM");
    }
    QString currLogFilePath = logDir + "/";
    QDir dir;
    dir.mkpath(currLogFilePath);
    if (logFileName.isEmpty()) {
        logFileName += dtmCur.toString("yyyyMMdd_HHmmss_");
        logFileName += app->applicationName() + QString("(%1).txt").arg(app->applicationPid());
    } else {
        logFileName = dtmCur.toString("yyyyMMdd_HHmmss_");
        logFileName += m_logFileName;
    }
    currLogFilePath += logFileName;

    static QFile s_logFile;
    if (m_logFile) {
        if (m_logFile->isOpen()==true)
            m_logFile->close();
    }

    m_logFile = &s_logFile;
    m_logFile->setFileName(currLogFilePath);
    if (m_logFile->open(QIODevice::WriteOnly) == false) {
        m_logFile = NULL;
    } else {
        ret = true;
    }

    return ret;
}

QFile *QAppLogging::logFile()
{
    if (!m_logFile || (m_logFile->pos() >= m_maxFileSize)) {
        createLogFile();
    }

    return m_logFile;
}

void QAppLogging::setLogFilePath(const QString &fileName, const QString &fileDir)
{
    setLogFileDir(fileDir);
    setLogFileName(fileName);
}

void QAppLogging::setOutputDest(int value)
{
    if (m_logFile && (!(m_outputDest & eDestFile)) &&
        (value & eDestFile)) {
        createLogFile();
    }

    m_outputDest = value;
}

void QAppLogging::registerCategory(const char *category, QtMsgType severityLevel)
{
    Q_UNUSED(severityLevel);
    QAppCategoryOptions options(category, true);
    _registeredCategories << options;
}

QStringList QAppLogging::registeredCategories()
{
    QStringList sl;
    foreach (auto options, _registeredCategories) {
        sl.append(options.name);
    }
    return sl;
}

void QAppLogging::setCategoryLoggingOn(const QString &category, bool enable)
{
    QList<QAppCategoryOptions>::iterator it;
    QList<QAppCategoryOptions>::iterator end = _registeredCategories.end();
    for (it = _registeredCategories.begin(); it != end; it++) {
        if (it->name == category) {
            it->isEnable = enable;
            break;
        }
    }

    return;
}

bool QAppLogging::categoryLoggingOn(const QString &category)
{
    bool enable = false;
    foreach (auto options, _registeredCategories) {
        if (options.name == category) {
            enable = options.isEnable;
            break;
        }
    }

    return enable;
}

void QAppLogging::setFilterRulesByLevel(LogLevel severityLevel)
{
    QString filterRules;

    filterRules += QString("*") + QAL_TAG_TAIL + ".debug=false\n";
    filterRules += QString("*") + QAL_TAG_TAIL + ".info=false\n";
    filterRules += QString("*") + QAL_TAG_TAIL + ".warning=false\n";
    filterRules += QString("*") + QAL_TAG_TAIL + ".critical=false\n";
    filterRules += QString("*") + QAL_TAG_TAIL + ".fatal=false\n";

    foreach (auto options, _registeredCategories) {
        QString &category = options.name;
        bool isCategoryEnable = options.isEnable;
        if (!isCategoryEnable) {
            continue;
        }

        if (severityLevel <= TraceLevel) {
            filterRules += category;
            filterRules += ".debug=true\n";
        }
        if (severityLevel <= DebugLevel) {
            if (!category.contains(QAL_TAG_TRACE QAL_TAG_TAIL)) {
                filterRules += category;
                filterRules += ".debug=true\n";
            }
        }
        if (severityLevel <= InfoLevel) {
            filterRules += category;
            filterRules += ".info=true\n";
        }
        if (severityLevel <= WarnLevel) {
            filterRules += category;
            filterRules += ".warning=true\n";
        }
        if (severityLevel <= ErrorLevel) {
            filterRules += category;
            filterRules += ".critical=true\n";
        }
        if (severityLevel <= FatalLevel) {
            filterRules += category;
            filterRules += ".fatal=true\n";
        }
    }

    qDebug() << "Filter rules" << filterRules;
    QLoggingCategory::setFilterRules(filterRules);
}

