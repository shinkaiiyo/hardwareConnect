#include "lhardwareconnect.h"
#include <QThread>
#include "lconnectHardware.h"
#include <QSerialPortInfo>
#include <QStringList>
#include <QDebug>

LhardwareConnect::LhardwareConnect(QWidget *parent) :
    QObject(parent), m_threadForSerial(new QThread(this))
{
    qDebug() << "con create";
    m_port = new SerialPort();
    // for serial
    connect(m_port,            SIGNAL(sgConnected()),                   this,   SLOT(onConnected()));
    connect(m_port,            SIGNAL(sgDisconnected()),                this,   SLOT(onDisconnected()));
    connect(m_port,            SIGNAL(sgReceivedData(QByteArray)),      this,   SLOT(onRecvedData(QByteArray) ));
    connect(m_port,            SIGNAL(sgReceivedErrorData(QByteArray)), this,   SLOT(onRecvedErrorData(QByteArray)));
    connect(m_threadForSerial, SIGNAL(finished()),                      m_port, SLOT(deleteLater()));
    connect(m_threadForSerial, SIGNAL(started()),                       m_port, SLOT(init()));
    connect(this,              SIGNAL(signalSendData(QByteArray)),      m_port, SLOT(sendData(QByteArray)));
    m_port->moveToThread(m_threadForSerial);
    m_threadForSerial->start(QThread::TimeCriticalPriority);

    qDebug() << "con moved";
}

LhardwareConnect::~LhardwareConnect()
{
    m_threadForSerial->exit();
    delete m_threadForSerial;
    m_threadForSerial = NULL;
    delete m_port;
    m_port = NULL;
}

void LhardwareConnect::createConnection()
{
    QStringList serialportinfo;
    foreach(QSerialPortInfo info,QSerialPortInfo::availablePorts())
    {
        serialportinfo<<info.portName();
    }
    QMetaObject::invokeMethod(m_port, "tryConnect", Qt::QueuedConnection ,
                              Q_ARG(QString, "COM5"),
                              Q_ARG(quint32, 115200),
                              Q_ARG(quint32, 8),
                              Q_ARG(quint32, 0),
                              Q_ARG(quint32, 1)
                                     );
}

void LhardwareConnect::close()
{
    m_threadForSerial->exit();
    m_port->close();
}

void LhardwareConnect::onRecvedData(QByteArray data)
{
    qDebug() << "data is " << data;
    emit signalSensorInfo(data);
}


void LhardwareConnect::onRecvedErrorData(QByteArray data)
{
    qDebug()  << "connect and error data is " << data.toHex();
}

void LhardwareConnect::onSendData(QByteArray data)
{
    qDebug() << "data is " << data;
    emit signalSendData(data);
}

void LhardwareConnect::onConnected()
{
    emit signalConnected();
}

void LhardwareConnect::onDisconnected()
{
    emit signalDisConnected();
}
