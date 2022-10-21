#include "lconnectHardware.h"
#include <QDebug>

SerialPort::SerialPort(QObject *parent) :
    QObject(parent),
    m_port(new QSerialPort(this) ),
    m_timerResponseTimeout(new QBasicTimer()),
    m_timerRTU35Timeout(new QBasicTimer() ),
    m_timerElapsedRTUTimeout(new QElapsedTimer() )
{
    setT15IntervalUsec(750);
    setT35IntervalUsec(1500);
    m_lastSendPktSize = 0;
    qRegisterMetaType<QSerialPort::SerialPortError>("QSerialPort::SerialPortError");
    m_isIgnoreAck = false;
    setResponseTimeout(10);
}

void SerialPort::init()
{
    qDebug() << "con init";
    createConnection();
    createState();
}

SerialPort::~SerialPort()
{
    if( m_timerResponseTimeout != 0 )
        delete m_timerResponseTimeout;

    if( m_timerElapsedRTUTimeout != 0 )
        delete m_timerElapsedRTUTimeout;

    if( m_timerRTU35Timeout != 0 )
        delete m_timerRTU35Timeout;
    m_port->close();
    delete m_port;
    m_port = NULL;
}

void SerialPort::createConnection()
{
    connect(m_port,             SIGNAL(bytesWritten(qint64)),           this,               SLOT(onBytesWritten(qint64))        );
    connect(m_port,             SIGNAL(readyRead()),                    this,               SLOT(onReadyRead())                 );
    connect(m_port,             SIGNAL(error(QSerialPort::SerialPortError)), this,          SLOT(onError(QSerialPort::SerialPortError) ));
}


void SerialPort::createState()
{

    qDebug() << "con create state";
    m_state = new QStateMachine(this);

    QState *mainState = new QState(m_state);
    QFinalState *finalState = new QFinalState(m_state);

    QState *init = new QState(mainState);
    QState *connected = new QState(mainState);
    QState *disconnected = new QState(mainState);

    QState *ready = new QState(connected);
    QState *sending = new QState(connected);
    QState *waitAck = new QState(connected);

    m_state->setInitialState(mainState);

    mainState->setInitialState(init);
    connected->setInitialState(ready);

    mainState->addTransition(this,          SIGNAL(sgStateStop()            ),  finalState);

    init->addTransition(this,               SIGNAL(sgInitOk()               ),  connected);
    init->addTransition(this,               SIGNAL(sgReInit()               ),  init );

    disconnected->addTransition(this,       SIGNAL(sgConnected()           ),  connected );

    connected->addTransition(this,          SIGNAL(sgTryDisconnect()        ),  disconnected );
    connected->addTransition(this,          SIGNAL(sgError(QString)         ),  disconnected );

    ready->addTransition(this,              SIGNAL(sgSendPktQueued()),          ready);
    ready->addTransition(this,              SIGNAL(sgSendData()      ),         sending);

    sending->addTransition(this,            SIGNAL(sgSendingComplete()      ),  waitAck);
    sending->addTransition(this,            SIGNAL(sgIgnoreAck()            ),  ready);

    waitAck->addTransition(this,            SIGNAL(sgReponseReceived()          ),  ready);
    waitAck->addTransition(this,            SIGNAL(sgResponseTimeout()           ),  ready);

    connect(mainState,      SIGNAL(entered()), this, SLOT(mainStateEntered()        ));
    connect(finalState,     SIGNAL(entered()), this, SLOT(finalStateEntered()       ));

    connect(init,           SIGNAL(entered()), this, SLOT(initEntered()             ));
    connect(connected,      SIGNAL(entered()), this, SLOT(connectedEntered()        ));
    connect(disconnected,   SIGNAL(entered()), this, SLOT(disconnectedEntered()     ));

    connect(ready,          SIGNAL(entered()), this, SLOT(readyEntered()            ));

    connect(sending,        SIGNAL(entered()), this, SLOT(sendingEntered()          ));
    connect(waitAck,        SIGNAL(entered()), this, SLOT(waitAckEntered()          ));

    m_state->start();

    qDebug() << "con state created";
}

void SerialPort::mainStateEntered(){}

void SerialPort::finalStateEntered()
{
    qDebug() << Q_FUNC_INFO;
}

void SerialPort::initEntered()
{
    qDebug() << Q_FUNC_INFO;
    emit sgInitOk();
}
void SerialPort::disconnectedEntered()
{
    qDebug() << Q_FUNC_INFO << "is port open?" << m_port->isOpen() <<
                m_port->portName() << m_port->baudRate() << m_port->handle() << m_port->parity();
    if( m_port->isOpen() == true )
    {
        m_port->close();
    }
    m_timerResponseTimeout->stop();

    if( m_recvPkt.isEmpty() == false )
    {
        emit sgReceivedErrorData(m_recvPkt);
    }
    m_queueSendPkt.clear();
    m_recvPkt.clear();



}

void SerialPort::connectedEntered()
{
    qDebug() << Q_FUNC_INFO << "is port open?" << m_port->isOpen() <<
                m_port->portName() <<
                m_port->baudRate() <<
                m_port->dataBits() <<
                m_port->parity() <<
                m_port->stopBits() <<
                m_port->handle() ;

}

void SerialPort::readyEntered()
{
    qDebug() << Q_FUNC_INFO;
    emit sgReadyEntered();
    if( m_queueSendPkt.count() )
    {
        m_lastSendPkt = m_queueSendPkt.takeFirst();
        m_lastSendPktSize = m_lastSendPkt.size();
        emit sgSendData();
    }
}

void SerialPort::sendingEntered()
{
    qDebug() << Q_FUNC_INFO << QString::fromStdString(m_lastSendPkt.toStdString());
    m_port->write(m_lastSendPkt);
}

void SerialPort::waitAckEntered()
{
    m_timerResponseTimeout->start(m_msec_responseTimeout, Qt::PreciseTimer, this);
}


bool SerialPort::tryConnect(QString portName, quint32 baudrate, quint32 dataBits, quint32 parity, quint32 stopBits )
{
    m_port->setPortName(portName);

    m_port->setBaudRate(baudrate);
    m_port->setDataBits((QSerialPort::DataBits)dataBits);
    m_port->setParity((QSerialPort::Parity)parity);
    m_port->setStopBits((QSerialPort::StopBits)stopBits);
    m_port->setFlowControl(QSerialPort::NoFlowControl);

    if( m_port->open(QIODevice::ReadWrite) == true )
    {
        qDebug() << "try connect true";
        emit sgConnected();
        return true;
    }
    else
    {
        qDebug() << "try connect false";
        emit sgDisconnected();
        return false;
    }
}

void SerialPort::tryDisconnect()
{
    if( m_port->isOpen() == true )
    {
        m_port->close();
    }

    emit sgTryDisconnect();
}

void SerialPort::changeBaudrate(quint32 baudrate )
{
    m_port->setBaudRate(baudrate);
    const int multiplier = 1;
    if( baudrate > QSerialPort::Baud19200 )
    {
        // defined in modbus-rtu specification.
        setT15IntervalUsec(750 * multiplier);
        setT35IntervalUsec(1500 * multiplier);
    }
    else
    {
        int oneBytesBits = (1 +
                       m_port->dataBits() +
                       (m_port->parity() == QSerialPort::NoParity ? 0 : 1) +
                       (m_port->stopBits() == QSerialPort::OneStop ? 1 : 2 ) );

        setT15IntervalUsec( (1000000 / (m_port->baudRate()  / oneBytesBits)) * 1.5  * multiplier);
        setT35IntervalUsec( (1000000 / (m_port->baudRate() / oneBytesBits)) * 3.5 * multiplier);
    }
}
void SerialPort::changeDataBits(quint32 dataBits)
{
    m_port->setDataBits((QSerialPort::DataBits) dataBits);
}
void SerialPort::changeParity(quint32 parity)
{
    m_port->setParity((QSerialPort::Parity) parity );
}
void SerialPort::changeStopBits(quint32 stopBits)
{
    m_port->setStopBits((QSerialPort::StopBits) stopBits);
}

QString SerialPort::errorString()
{
    return QString("[%1] [error num: %2]")
            .arg( m_port->errorString() )
            .arg(m_port->error());

}


void SerialPort::sendData(QByteArray sendData)
{
    QByteArray temp = sendData;
    QByteArray ttemp = sendData.remove(0, 4);
    temp.append(ModbusHigh(ttemp, temp.at(3))).append(ModbusLow(ttemp, temp.at(3)));
    qDebug() << "send byte is" << temp;
    m_port->write(temp);
}

void SerialPort::onError(QSerialPort::SerialPortError serialError)
{
    if( serialError == QSerialPort::NoError ||
        serialError == QSerialPort::ParityError  ||
        serialError == QSerialPort::FramingError ||
        serialError == QSerialPort::UnknownError )
        return;

    qDebug() << Q_FUNC_INFO << "errorNumber" << serialError <<
                m_port->portName() <<
                m_port->baudRate() <<
                m_port->dataBits() <<
                m_port->stopBits() <<
                m_port->parity() <<
                m_port->handle() <<
                m_port->errorString();

    emit sgError(m_port->errorString() );
}

void SerialPort::onBytesWritten(qint64 count)
{
    m_lastSendPktSize -= count;
    if( m_lastSendPktSize == 0)
    {
        emit sgSendedData(m_lastSendPkt);
        if( isIgnoreAck() == true )
        {
            emit sgIgnoreAck();
        }
        else{
            emit sgSendingComplete();
        }

    }
}

void SerialPort::onReadyRead()
{
    QByteArray buffer;
    qDebug() << "read buffer data";
    while(m_port->bytesAvailable())
    {
        m_timerResponseTimeout->start(m_msec_responseTimeout, Qt::PreciseTimer, this);
        buffer = m_port->read(1);
        m_recvPkt += buffer;
        if(check_ModbusRTU_CRC16(m_recvPkt) == true )
        {
            emit sgReponseReceived();
            emit sgReceivedData(m_recvPkt);
            m_timerResponseTimeout->stop();
            m_recvPkt.clear();
        }
        else if( m_recvPkt.count() > 255 )
        {
            emit sgReponseReceived();
            emit sgReceivedErrorData(m_recvPkt);
            m_timerResponseTimeout->stop();
            m_recvPkt.clear();
        }
    }

}

void SerialPort::timerEvent(QTimerEvent *event)
{
    if ( event->timerId() == m_timerResponseTimeout->timerId())
    {
        m_timerResponseTimeout->stop();
        m_timerRTU35Timeout->stop();

        emit sgResponseTimeout();

        if( m_recvPkt.isEmpty() == false )
        {
            emit sgReceivedErrorData(m_recvPkt);
            m_recvPkt.clear();
        }
    }
    else if ( event->timerId() == m_timerRTU35Timeout->timerId())
    {
        m_timerResponseTimeout->stop();
        m_timerRTU35Timeout->stop();

    }
    else {
        QObject::timerEvent(event);
    }
}


bool SerialPort::check_ModbusRTU_CRC16(QByteArray buf)
{
    if(buf.size() >= 4)
    {
        qDebug() << "buffer size is " << buf.size();
        QByteArray srcCrc = buf.mid(buf.size()-2, 2);
        QByteArray dstCrc = "";
        quint16 crc = ModbusRTU_CRC16(buf.remove(buf.size()-2, 2).remove(0, 4), buf.at(3));
        dstCrc = QByteArray::fromRawData((char*)&crc, 2);

        qDebug() << "src is " << srcCrc;
        qDebug() << "dst is " << dstCrc;
        if( srcCrc == dstCrc )
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}

quint16 SerialPort::ModbusRTU_CRC16 (const char *buf, quint16 wLength)
{
    quint8 index;
    quint16 check16=0;
    quint8 crc_low=0XFF;
    quint8 crc_high=0XFF;
    while(wLength--)
    {
        index=crc_high^(*buf++);
        crc_high=crc_low^CRC16HiTable[index];
        crc_low=CRC16LoTable[index];
    }
    check16 += crc_low;
    check16 <<= 8;
    check16 += crc_high;
    return check16;
}

quint8 SerialPort::ModbusLow(const char *buf, quint16 wLength)
{
    quint8 index;
    quint16 check16=0;
    quint8 crc_low=0XFF;
    quint8 crc_high=0XFF;
    while(wLength--)
    {
        index=crc_high^(*buf++);
        crc_high=crc_low^CRC16HiTable[index];
        crc_low=CRC16LoTable[index];
    }
    return crc_low;
}

quint8 SerialPort::ModbusHigh(const char *buf, quint16 wLength)
{
    quint8 index;
    quint16 check16=0;
    quint8 crc_low=0XFF;
    quint8 crc_high=0XFF;
    while(wLength--)
    {
        index=crc_high^(*buf++);
        crc_high=crc_low^CRC16HiTable[index];
        crc_low=CRC16LoTable[index];
    }
    return crc_high;
}

void SerialPort::close()
{
    m_port->close();
}


