#include "lconnectHardware.h"
#include "ecode.h"
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
    setResponseTimeout(5);
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
    qDebug() << "send byte is" << sendData;
    m_port->write(sendData);
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
    qDebug() << "read buffer data";
    while(m_port->bytesAvailable())
    {
        m_timerResponseTimeout->start(m_msec_responseTimeout, Qt::PreciseTimer, this);
        uart1_rx.temp_buf = m_port->read(1).toUInt();
        if(uart1_rx.temp_buf == USART_START_MARK1 && uart1_rx.rcv_state == 0)
        {
            uart1_rx.rcv_state = 1;
            uart1_rx.rcv_len = 1;
            uart1_rx.rcv_buf[0] = USART_START_MARK1;
        }
        else if(uart1_rx.temp_buf != USART_START_MARK1 && uart1_rx.rcv_state == 0)
        {
            uart1_rx.rcv_state = 0;
            uart1_rx.rcv_len = 0;
        }
        else if(uart1_rx.temp_buf == USART_START_MARK2 && uart1_rx.rcv_state == 1)
        {
            uart1_rx.rcv_state = 2;
            uart1_rx.rcv_len = 2;
            uart1_rx.rcv_buf[1] = USART_START_MARK2;
        }
        else if(uart1_rx.temp_buf != USART_START_MARK2 && uart1_rx.rcv_state == 1 )
        {
            uart1_rx.rcv_state = 0;
            uart1_rx.rcv_len = 0;
        }
        else if(uart1_rx.rcv_len < uart1_rx.rcv_frame_len-1 && uart1_rx.rcv_state == 2)
        {
            uart1_rx.rcv_buf[uart1_rx.rcv_len] = uart1_rx.temp_buf;
            if(uart1_rx.rcv_len == 4)
            {
                if(uart1_rx.rcv_buf[2] == 0x00)
                    uart1_rx.rcv_frame_len = uart1_rx.rcv_buf[4] + 9;
                else
                    uart1_rx.rcv_frame_len = uart1_rx.rcv_buf[4];
            }
            uart1_rx.rcv_state = 2;
            uart1_rx.rcv_len++;
        }
        else if(uart1_rx.rcv_len == uart1_rx.rcv_frame_len-1 && uart1_rx.rcv_state == 2)
        {
            uart1_rx.rcv_buf[uart1_rx.rcv_len] = uart1_rx.temp_buf;

            /* init para*/
            uart1_rx.rcv_state = 0;
            uart1_rx.rcv_len = 0;
            uart1_rx.rcv_frame_len = MAX_PROTOBUF_LENGTH;

        }
        else
        {
            uart1_rx.rcv_state = 0;
            uart1_rx.rcv_len = 0;
        }
        uart1_rx.temp_buf = 0x00;
        uart1_rx.time_counter = 0;
        m_timerResponseTimeout->stop();
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

void SerialPort::close()
{
    m_port->close();
}


uint8_t SerialPort::CRC8_Table(uint8_t *p, char counter)
{
    unsigned char crc8 = 0;
    for (; counter > 0; counter--)
    {
        crc8 = crc_array[crc8^*p]; //CRC-8
        p++;
    }
    return crc8;
}

uint8_t SerialPort::checkProtocolFrameTypedefLegal(DATA_ProtocolFrameTypedef *proto)
{
    if( (proto -> current_guidewire_position > UPPER_LIMITS_CURRENT_GUIDEWIRE_POSITION) || \
        (proto -> current_guidewire_position < LOWER_LIMITS_CURRENT_GUIDEWIRE_POSITION) 	)
        return ERROR_OUTOF_RANGE_LINEAR_GUIDEWIRE;
    if( (proto -> current_guidewire_angle		 > UPPER_LIMITS_CURRENT_GUIDEWIRE_ANGLE   ) || \
        (proto -> current_guidewire_angle		< LOWER_LIMITS_CURRENT_GUIDEWIRE_ANGLE   ) 	)
        return ERROR_OUTOF_RANGE_ROTATION_GUIDEWIRE;
    if( (proto -> current_tube_position			 > UPPER_LIMITS_CURRENT_TUBE_POSITION		  ) || \
        (proto -> current_tube_position			< LOWER_LIMITS_CURRENT_TUBE_POSITION		 )	)
        return ERROR_OUTOF_RANGE_LINEAR_TUBE;
    if( (proto -> current_tube_angle				 > UPPER_LIMITS_CURRENT_TUBE_ANGLE        ) || \
        (proto -> current_tube_angle				  < LOWER_LIMITS_CURRENT_TUBE_ANGLE        ) 	)
        return ERROR_OUTOF_RANGE_ROTATION_TUBE;
    if( (proto -> current_ballon_position		 > UPPER_LIMITS_CURRENT_BALLON_POSITION   ) || \
        (proto -> current_ballon_position	  < LOWER_LIMITS_CURRENT_BALLON_POSITION   ) 	)
        return ERROR_OUTOF_RANGE_LINEAR_BALLON;

    if( (__GET_2BIT(proto ->mode, 0) !=0x00) || (__GET_2BIT(proto ->mode, 0) !=0x01))
        return ERROR_MOTION_MODE;
    if( (__GET_2BIT(proto ->mode, 2) !=0x00) || (__GET_2BIT(proto ->mode, 2) !=0x01))
        return ERROR_MOTION_MODE;
    if( (__GET_2BIT(proto ->mode, 4) !=0x00) || (__GET_2BIT(proto ->mode, 4) !=0x01))
        return ERROR_MOTION_MODE;

    return 0;
}

uint8_t SerialPort::packProtocolFrame(DATA_ProtocolFrameTypedef *proto)
{
    uint8_t i;
    uint16_t crc_check_val;
    uint8_t errCode;
    errCode = checkProtocolFrameTypedefLegal(proto);
    if(errCode)
        return errCode;
    i = 11;
    uart1_tx_buf[0] = 0xff; //Frame header
    uart1_tx_buf[1] = 0x00; //Frame header
    uart1_tx_buf[2] = proto->address;
    uart1_tx_buf[3] = proto->mode;

    /* uart1_tx_buf[4] is the length, end of function assigment it */
    uart1_tx_buf[5] = proto->button_state;
    uart1_tx_buf[6] = proto->guidewire_linear_bar;
    uart1_tx_buf[7] = proto->guidewire_rot_bar;
    uart1_tx_buf[8] = proto->tube_linear_bar;
    uart1_tx_buf[9] = proto->tube_rot_bar;
    uart1_tx_buf[10] = proto->ballon_linear_bar;

    if ( __IS_BITx_SET(proto->mode, 6) )
    {
        uart1_tx_buf[i] = proto->maxSpeed_guidewire_linear_bar; i++;
        uart1_tx_buf[i] = proto->maxSpeed_guidewire_rot_bar; i++;
        uart1_tx_buf[i] = proto->maxSpeed_tube_linear_bar; i++;
        uart1_tx_buf[i] = proto->maxSpeed_tube_rot_bar; i++;
        uart1_tx_buf[i] = proto->maxSpeed_ballon_linear_bar; i++;

        uart1_tx_buf[i] = proto->step_guidewire_linear_bar; i++;
        uart1_tx_buf[i] = proto->step_guidewire_rot_bar; i++;
        uart1_tx_buf[i] = proto->step_tube_linear_bar; i++;
        uart1_tx_buf[i] = proto->step_tube_rot_bar; i++;
        uart1_tx_buf[i] = proto->step_ballon_linear_bar; i++;
    }

    if ( __IS_BITx_SET(proto->mode, 7) )
    {
        uart1_tx_buf[i] = ((proto->current_guidewire_position) >> 8) & 0xff; i++;
        uart1_tx_buf[i] = (proto->current_guidewire_position) & 0xff; i++;
        uart1_tx_buf[i] = ((proto->current_guidewire_angle) >> 8) & 0xff; i++;
        uart1_tx_buf[i] = (proto->current_guidewire_angle) & 0xff; i++;

        uart1_tx_buf[i] = ((proto->current_tube_angle) >> 8) & 0xff; i++;
        uart1_tx_buf[i] = (proto->current_tube_angle) & 0xff; i++;
        uart1_tx_buf[i] = ((proto->current_tube_position) >> 8) & 0xff; i++;
        uart1_tx_buf[i] = (proto->current_tube_position) & 0xff; i++;

        uart1_tx_buf[i] = ((proto->current_ballon_position) >> 8) & 0xff; i++;
        uart1_tx_buf[i] = (proto->current_ballon_position) & 0xff; i++;
    }
    uart1_tx_buf[i] = proto->token_to_next_address; i++;

    proto->frame_len = i + 4;
    uart1_tx_buf[4] = proto->frame_len;

    //CRC-8
    crc_check_val = CRC8_Table(&uart1_tx_buf[2], i - 3);
    proto->crc_check = crc_check_val;
    uart1_tx_buf[i] = (crc_check_val >> 8) & 0xff; i++;
    uart1_tx_buf[i] = (crc_check_val) & 0xff; i++;

    uart1_tx_buf[i] = 0x00; i++;
    uart1_tx_buf[i] = 0xff; i++;


    return 0;
}

uint8_t SerialPort::packCMDFrame(CMD_ProtocolFrameTypedef *proto)
{
    uint8_t i, j;// i: buf pointer; j: circle countnum
    uint16_t crc_check_val;
    i = 2;
    uart1_tx_buf[0] = 0xff; //Frame header
    uart1_tx_buf[1] = 0x00; //Frame header

    uart1_tx_buf[i] = proto->address; i++;
    uart1_tx_buf[i] = proto->target_address; i++;
    uart1_tx_buf[i] = proto->data_length; i++;
    uart1_tx_buf[i] = proto->mode; i++;

    for (j = 0; j < (proto->data_length); j++)
    {
        uart1_tx_buf[i] = proto->data[j]; i++;
    }

    //CRC-8
    crc_check_val = CRC8_Table(&uart1_tx_buf[2], i - 2);
    proto->crc_check = crc_check_val;
    uart1_tx_buf[i] = (crc_check_val >> 8) & 0xff; i++;
    uart1_tx_buf[i] = (crc_check_val) & 0xff; i++;

    uart1_tx_buf[i] = 0x00; i++;
    uart1_tx_buf[i] = 0xff; i++;

    return i;
}

uint8_t SerialPort::unpackProtocolFrame(DATA_ProtocolFrameTypedef *master, CMD_ProtocolFrameTypedef *CMD)
{
    uint8_t i, j;
    uint16_t crc_check_val;
    uint16_t crc_recv_val;

    i = 2;
    if (uart1_rx.rcv_buf[i] == 0) //Slave Device: Get CMD mode
    {
        CMD->address = uart1_rx.rcv_buf[i];           i++;
        CMD->target_address = uart1_rx.rcv_buf[i]; 		i++;
        CMD->data_length = uart1_rx.rcv_buf[i];       i++;
        CMD->mode = uart1_rx.rcv_buf[i]; 	i++;
        for (j = 0; j < (CMD->data_length); j++)
        {
            CMD->data[j] = uart1_rx.rcv_buf[i];
            i++;
        }
        crc_check_val = CRC8_Table(&uart1_rx.rcv_buf[2], i - 2);
        crc_recv_val = (uart1_rx.rcv_buf[i] << 8) + uart1_rx.rcv_buf[i + 1]; i++;
        if (crc_check_val != crc_recv_val)
            return ERROR_CRC_CHECK;
        return UNPACK_CMD_OK;
    }
    else
    {

        master->address = uart1_rx.rcv_buf[2];
        master->mode = uart1_rx.rcv_buf[3];
        master->frame_len = uart1_rx.rcv_buf[4];

        master->button_state = uart1_rx.rcv_buf[5];

        master->guidewire_linear_bar = uart1_rx.rcv_buf[6];
        master->guidewire_rot_bar = uart1_rx.rcv_buf[7];
        master->tube_linear_bar = uart1_rx.rcv_buf[8];
        master->tube_rot_bar = uart1_rx.rcv_buf[9];
        master->ballon_linear_bar = uart1_rx.rcv_buf[10];
        i = 11;
        if (uart1_rx.rcv_buf[3] & 0x40)
        {
            master->maxSpeed_guidewire_linear_bar = uart1_rx.rcv_buf[i]; 	i++;
            master->maxSpeed_guidewire_rot_bar = uart1_rx.rcv_buf[i]; 	i++;
            master->maxSpeed_tube_linear_bar = uart1_rx.rcv_buf[i]; 	    i++;
            master->maxSpeed_tube_rot_bar = uart1_rx.rcv_buf[i]; 	    i++;
            master->maxSpeed_ballon_linear_bar = uart1_rx.rcv_buf[i];      i++;

            master->step_guidewire_linear_bar = uart1_rx.rcv_buf[i];     i++;
            master->step_guidewire_rot_bar = uart1_rx.rcv_buf[i];		i++;
            master->step_tube_linear_bar = uart1_rx.rcv_buf[i];		    i++;
            master->step_tube_rot_bar = uart1_rx.rcv_buf[i];		    i++;
            master->step_ballon_linear_bar = uart1_rx.rcv_buf[i];  	    i++;
        }

        if (uart1_rx.rcv_buf[3] & 0x80)
        {
            master->current_guidewire_position = (uart1_rx.rcv_buf[i] << 8) + uart1_rx.rcv_buf[i + 1]; i = i + 2;
            master->current_guidewire_angle = (uart1_rx.rcv_buf[i] << 8) + uart1_rx.rcv_buf[i + 1]; i = i + 2;
            master->current_tube_position = (uart1_rx.rcv_buf[i] << 8) + uart1_rx.rcv_buf[i + 1]; i = i + 2;
            master->current_tube_angle = (uart1_rx.rcv_buf[i] << 8) + uart1_rx.rcv_buf[i + 1]; i = i + 2;
            master->current_ballon_position = (uart1_rx.rcv_buf[i] << 8) + uart1_rx.rcv_buf[i + 1]; i = i + 2;
        }
        master->token_to_next_address = uart1_rx.rcv_buf[i]; i++;
        //CRC-8
        crc_check_val = CRC8_Table(&uart1_rx.rcv_buf[2], i - 3);
        master->crc_check = crc_check_val;
        crc_recv_val = (uart1_rx.rcv_buf[i] << 8) + uart1_rx.rcv_buf[i + 1]; i++;
        if (crc_check_val != crc_recv_val)
            return ERROR_CRC_CHECK;
        return UNPACK_DATA_OK;
    }
    return 0;
}

void SerialPort::updateControlPart()
{
#if DEVICE_ID == DEVICE_ID_MASTER
    tx_data.button_state = masterPanelData.Turbo_current_state;
    tx_data.ballon_linear_bar = *masterPanelData.ballon_fnb_bar;
    tx_data.guidewire_linear_bar = *masterPanelData.guidewire_fnb_bar;
    tx_data.guidewire_rot_bar = *masterPanelData.guidewire_rot_bar;
    tx_data.tube_linear_bar = *masterPanelData.tube_fnb_bar;
    tx_data.tube_rot_bar = *masterPanelData.tube_rot_bar;
#else
    tx_data.button_state = rx_data.button_state;
    tx_data.ballon_linear_bar = rx_data.step_ballon_linear_bar;
    tx_data.guidewire_linear_bar = rx_data.guidewire_linear_bar;
    tx_data.guidewire_rot_bar = rx_data.guidewire_rot_bar;
    tx_data.tube_linear_bar = rx_data.tube_linear_bar;
    tx_data.tube_rot_bar = rx_data.tube_rot_bar;
#endif
}

void SerialPort::updateImformationPart()
{
#if DEVICE_ID == DEVICE_ID_MASTER
    tx_data.current_guidewire_position = rx_data.current_guidewire_position;
    tx_data.current_guidewire_angle = rx_data.current_guidewire_angle;
    tx_data.current_tube_position = rx_data.current_tube_position;
    tx_data.current_tube_angle = rx_data.current_tube_angle;
    tx_data.current_ballon_position = rx_data.current_ballon_position;
#endif
#if DEVICE_ID == DEVICE_ID_ACTUATOR
/** value of encoder data @todo **/
    tx_data.current_guidewire_position = rx_data.current_guidewire_position;
    tx_data.current_guidewire_angle = rx_data.current_guidewire_angle;
    tx_data.current_tube_position = rx_data.current_tube_position;
    tx_data.current_tube_angle = rx_data.current_tube_angle;
    tx_data.current_ballon_position = rx_data.current_ballon_position;
#endif
}


void SerialPort::CMD_Handle()
{
    if(__IS_CMD_CALLING_ME(cmd_rx_data.target_address))
    {
        switch(cmd_rx_data.mode)
        {
            #if DEVICE_ID != DEVICE_ID_MASTER
                case CMD_HALT_ALL:
                {
                    executeWhenReceiveCmdHaltAll();
                    break;
                }
                case CMD_SPECIFIED_SEND:
                {
                    executeWhenReceiveSpecifiedSend();
                    break;
                }
                case CMD_CHANGE_ADDRESS:
                {
                    executeWhenReceiveChangeAddress();
                    break;
                }
            #endif

            case CMD_PING:
            {
                executeWhenReceiveCmdPing();
                break;
            }
            case CMD_RESPONSE_PING:
            {
                executeWhenReceiveResponsePing();
                break;
            }
            case CMD_PARDON:
            {
                executeWhenReceivePardon();
                break;
            }

            #if DEVICE_ID == DEVICE_ID_MASTER
                case CMD_FEEDBACK_CHANGES:
                {
                    executeWhenReceiveFeedbackChanges();
                    break;
                }
            #endif
            default:
                    break;
        }
    }
}

void SerialPort::CMD_HaltAll()
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = DEVICE_ID_ALL;
    cmd_tx_data.data_length = 0;
    cmd_tx_data.mode = CMD_HALT_ALL;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING;
}

void SerialPort::CMD_SpecifiedSend(uint8_t Address)
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = Address;
    cmd_tx_data.data_length = 1;
    cmd_tx_data.mode = CMD_SPECIFIED_SEND;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING | DEVICE_STATE_WAITING_RECEIVING_CMD;
}

void SerialPort::CMD_ChangeAddress(uint8_t Address, uint8_t prevAddress, uint8_t nextAddress)
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = Address;
    cmd_tx_data.data_length = 3;
    cmd_tx_data.mode = CMD_CHANGE_ADDRESS;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    cmd_tx_data.data[1] = prevAddress;
    cmd_tx_data.data[2] = nextAddress;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING  | DEVICE_STATE_WAITING_RECEIVING_CMD;
}

void SerialPort::CMD_SpecifiedReady(uint8_t Address)
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = Address;
    cmd_tx_data.data_length = 1;
    cmd_tx_data.mode = CMD_SPECIFIED_READY;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING  | DEVICE_STATE_WAITING_RECEIVING_CMD;
}

void SerialPort::CMD_ResumeAll()
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = DEVICE_ID_ALL;
    cmd_tx_data.data_length = 0;
    cmd_tx_data.mode = CMD_HALT_ALL;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING;
}

void SerialPort::CMD_Ping(uint8_t Address)
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = Address;
    cmd_tx_data.data_length = 1;
    cmd_tx_data.mode = CMD_PING;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING  | DEVICE_STATE_WAITING_RECEIVING_CMD;
}

void SerialPort::CMD_ResponsePing(uint8_t Address)
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = Address;
    cmd_tx_data.data_length = 2;
    cmd_tx_data.mode = CMD_RESPONSE_PING;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    cmd_tx_data.data[1] = networkImformation.device_state;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING;
}

void SerialPort::CMD_Pardon()
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = networkImformation.previous_address;
    cmd_tx_data.data_length = 1;
    cmd_tx_data.mode = CMD_PARDON;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING  | DEVICE_STATE_WAITING_RECEIVING_DATA | DEVICE_STATE_WAITING_RECEIVING_CMD;
}

uint8_t SerialPort::CMD_PollingDeviceOnline()
{
    //todo
    return 0;
}

void SerialPort::executeWhenReceiveCmdPing()
{
    if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
    {
        networkImformation.reTransmit_count++;
        CMD_ResponsePing(cmd_rx_data.data[0]);
    }
    else
    {
        networkImformation.reTransmit_count = 0;
        networkImformation.device_state = DEVICE_STATE_ERROR;
        errCodeCallback(ERROR_DISCONNECTED);
    }
}

void SerialPort::executeWhenReceiveResponsePing()
{
    if(networkImformation.prev_sending_what == CMD_PING)
    {
        networkImformation.device_state = DEVICE_STATE_READY;
    }
    else
    {
        errCodeCallback(ERROR_MESSAGE_TYPE);
    }
}

void SerialPort::executeWhenReceivePardon()
{
    if(DEVICE_ID == DEVICE_ID_MASTER)
    {
        if(networkImformation.prev_sending_what == CMD_SPECIFIED_SEND)
        {
            errCodeCallback(ERROR_ENDLESS_LOOP);
            return;
        }
    }
    if(networkImformation.prev_sending_what == CMD_PARDON)
    {
        errCodeCallback(ERROR_ENDLESS_LOOP);
        return;
    }
    switch (networkImformation.prev_sending_what)
    {
    case CMD_HALT_ALL:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_HaltAll();
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    case CMD_PING:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_Ping(cmd_tx_data.target_address);
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    case CMD_RESPONSE_PING:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_ResponsePing(cmd_tx_data.target_address);
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    case CMD_CHANGE_ADDRESS:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_ChangeAddress(cmd_tx_data.target_address, cmd_tx_data.data[1], cmd_tx_data.data[2]);
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    case CMD_FEEDBACK_CHANGES:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_FeedbackChanges(cmd_tx_data.target_address);
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    case CMD_SPECIFIED_READY:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_SpecifiedReady(cmd_tx_data.target_address);
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    case CMD_RESPONSE_READY:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_ResponseSpecifiedReady(cmd_tx_data.target_address);
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    case CMD_RESUME_ALL:
    {
        if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
        {
            networkImformation.reTransmit_count++;
            CMD_ResumeAll();
        }
        else
        {
            networkImformation.reTransmit_count = 0;
            networkImformation.device_state = DEVICE_STATE_ERROR;
            errCodeCallback(ERROR_DISCONNECTED);
        }
    }
        break;
    default:
        break;
    }
}

void SerialPort::executeWhenReceiveResponseReady()
{
    if(	networkImformation.prev_sending_what == CMD_SPECIFIED_READY)
    {
        //setSalveOnline(cmd_tx_data.target_address); //todo
        networkImformation.device_state = DEVICE_STATE_READY;
    }
    else
    {
        errCodeCallback(ERROR_MESSAGE_TYPE);
    }
}

void SerialPort::executeWhenReceiveFeedbackChanges()
{
    if(	networkImformation.prev_sending_what == CMD_CHANGE_ADDRESS)
    {
        //updateDataLink(); //todo
        networkImformation.device_state = DEVICE_STATE_READY;
    }
    else
    {
        errCodeCallback(ERROR_MESSAGE_TYPE);
    }
}

void SerialPort::DATA_SendFlame(uint8_t Address)
{
    uint8_t DATA_SendFlame_errCode;
    networkImformation.device_state = DEVICE_STATE_SENDING_DATA ;
    tx_data.address = Address;
    updateControlPart();
    updateImformationPart();


    tx_data.token_to_next_address = networkImformation.following_address;

    DATA_SendFlame_errCode = packProtocolFrame(&tx_data);

    if (DATA_SendFlame_errCode != 0)
        errCodeCallback(DATA_SendFlame_errCode);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = PROTOCOL_DATA;
    networkImformation.device_state = DEVICE_STATE_SENDING_DATA ;
}

void SerialPort::DATA_HANDLE()
{
    if(rx_data.address == networkImformation.previous_address && rx_data.token_to_next_address == DEVICE_ID)
            DATA_SendFlame(networkImformation.following_address);
}

void SerialPort::resetActuator(uint8_t errorType)
{
    networkImformation.device_state = DEVICE_STATE_HALT;
        /** Halt all motor **/
        /** (Alarm) **/
     //todo   exectueStopAllMotor();
    switch (errorType)
    {
    case(ERROR_OUTOF_RANGE_LINEAR_BALLON):
    {
        tx_data.maxSpeed_ballon_linear_bar = rx_data.maxSpeed_ballon_linear_bar  | 0;
        tx_data.step_ballon_linear_bar = rx_data.step_ballon_linear_bar  | 0;
        break;
    }
    case(ERROR_OUTOF_RANGE_LINEAR_TUBE):
    {
        tx_data.maxSpeed_tube_linear_bar = rx_data.maxSpeed_tube_linear_bar | 0;
        tx_data.step_tube_linear_bar = rx_data.step_tube_linear_bar | 0;
        break;
    }
    case(ERROR_OUTOF_RANGE_LINEAR_GUIDEWIRE):
    {
        tx_data.maxSpeed_guidewire_linear_bar = rx_data.maxSpeed_guidewire_linear_bar | 0;
        tx_data.step_guidewire_linear_bar = rx_data.step_guidewire_linear_bar | 0;
        break;
    }
    case(ERROR_OUTOF_RANGE_ROTATION_TUBE):
    {
        tx_data.step_tube_rot_bar = rx_data.step_tube_rot_bar | 0;
        tx_data.maxSpeed_tube_rot_bar = rx_data.maxSpeed_tube_rot_bar | 0;
        break;
    }
    case(ERROR_OUTOF_RANGE_ROTATION_GUIDEWIRE):
    {
        tx_data.maxSpeed_guidewire_rot_bar = rx_data.maxSpeed_guidewire_rot_bar | 0;
        tx_data.step_guidewire_rot_bar = rx_data.step_guidewire_rot_bar | 0;
        break;
    }
    case(ERROR_FLAME_TIMEOUT):
    {
        CMD_Pardon();
        break;
    }
    case(ERROR_NO_BOARDCASE):
    {
        //todo
        break;
    }
    case(ERROR_MOTION_MODE):
    {
        //todo
        break;
    }
    }
}

void SerialPort::errCodeCallback(uint8_t errorCode)
{
    switch(errorCode)
    {
        case(ERROR_DISCONNECTED):
        {
            CMD_Ping(networkImformation.previous_address);
                break;
            }
        case(ERROR_OUTOF_RANGE_LINEAR_BALLON):
        {
            CMD_HaltAll();
            resetActuator(ERROR_OUTOF_RANGE_LINEAR_BALLON);
            break;
        }
        case(ERROR_OUTOF_RANGE_LINEAR_TUBE):
        {
            CMD_HaltAll();
            resetActuator(ERROR_OUTOF_RANGE_LINEAR_TUBE);
            break;
        }
        case(ERROR_OUTOF_RANGE_LINEAR_GUIDEWIRE):
        {
            CMD_HaltAll();
            resetActuator(ERROR_OUTOF_RANGE_LINEAR_GUIDEWIRE);
            break;
        }
        case(ERROR_OUTOF_RANGE_ROTATION_TUBE):
        {
            CMD_HaltAll();
            resetActuator(ERROR_OUTOF_RANGE_ROTATION_TUBE);
            break;
        }
        case(ERROR_OUTOF_RANGE_ROTATION_GUIDEWIRE):
        {
            CMD_HaltAll();
            resetActuator(ERROR_OUTOF_RANGE_ROTATION_GUIDEWIRE);
            break;
        }
        case(ERROR_FLAME_TIMEOUT):
        {
            CMD_HaltAll();
            resetActuator(ERROR_FLAME_TIMEOUT);
            break;
        }
        case(ERROR_NO_BOARDCASE):
        {
            CMD_HaltAll();
            resetActuator(ERROR_NO_BOARDCASE);
            break;
        }
        case(ERROR_MOTION_MODE):
        {
            CMD_HaltAll();
            resetActuator(ERROR_MOTION_MODE);
            break;
        }
        case(ERROR_CRC_CHECK):
        {
            CMD_Pardon();
            break;
        }
        case(ERROR_ENDLESS_LOOP):
        {
            if(DEVICE_ID == DEVICE_ID_MASTER)
            {
                CMD_HaltAll();
            }
            else
            {
                CMD_Ping(DEVICE_ID_MASTER);
            }
            break;
        }
        case(ERROR_MESSAGE_TYPE):
        {
            CMD_Pardon();
            break;
        }
        default:
            break;
    }
}

QByteArray SerialPort::array2ByteArray()
{
    QByteArray sendPkt;
    sendPkt.append((char*)uart1_tx_buf, sizeof(uint8_t) * MAX_PROTOBUF_LENGTH);
    return sendPkt;
}

void SerialPort::executeWhenReceiveCmdHaltAll()
{
    networkImformation.device_state = DEVICE_STATE_HALT;
    /** Halt all motor **/
    /** (Alarm) **/
    //todo  exectueStopAllMotor();
}

void SerialPort::executeWhenReceiveSpecifiedSend()
{
    if(networkImformation.reTransmit_count < MAX_RETRANSMIT_TIMES)
    {
        networkImformation.reTransmit_count++;
        CMD_ResponseSpecifiedSend(DEVICE_ID_MASTER);
    }
    else
    {
        networkImformation.reTransmit_count = 0;
        networkImformation.device_state = DEVICE_STATE_ERROR;
        errCodeCallback(ERROR_DISCONNECTED);
    }
}

void SerialPort::executeWhenReceiveChangeAddress()
{
    networkImformation.previous_address = cmd_tx_data.data[1];
    networkImformation.following_address = cmd_tx_data.data[2];   //todo
    if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
    {
        networkImformation.reTransmit_count++;
        CMD_FeedbackChanges(DEVICE_ID_MASTER);
    }
    else
    {
        networkImformation.reTransmit_count = 0;
        networkImformation.device_state = DEVICE_STATE_ERROR;
        errCodeCallback(ERROR_DISCONNECTED);
    }
}

void SerialPort::executeWhenReceiveSpecifiedReady()
{
    //todo link_into_datalink();
    if(networkImformation.reTransmit_count< MAX_RETRANSMIT_TIMES )
    {
        networkImformation.reTransmit_count++;
        CMD_ResponseSpecifiedReady(DEVICE_ID_MASTER);
    }
    else
    {
        networkImformation.reTransmit_count = 0;
        networkImformation.device_state = DEVICE_STATE_ERROR;
        errCodeCallback(ERROR_DISCONNECTED);
    }
}

void SerialPort::executeWhenReceiveResumeAll()
{
    networkImformation.device_state = DEVICE_STATE_STOP;
    // todo exectueStopAllMotor();
}

void SerialPort::CMD_ResponseSpecifiedSend(uint8_t Address)
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_DATA;
    DATA_SendFlame(Address);

    networkImformation.prev_sending_what = tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING;
}

void SerialPort::CMD_ResponseSpecifiedReady(uint8_t Address)
{
    uint8_t len = 0;
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    tx_data.address = 0;
    cmd_tx_data.target_address = Address;
    cmd_tx_data.data_length = 1;
    cmd_tx_data.mode = CMD_RESPONSE_READY;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    len = packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING;
}

void SerialPort::CMD_FeedbackChanges(uint8_t Address)
{
    networkImformation.device_state = DEVICE_STATE_SENDING_CMD;
    cmd_tx_data.address = 0;
    cmd_tx_data.target_address = Address;
    cmd_tx_data.data_length = 3;
    cmd_tx_data.mode = CMD_FEEDBACK_CHANGES;
    cmd_tx_data.data[0] = DEVICE_PROTOCOL_ADDRESS;
    cmd_tx_data.data[1] = networkImformation.previous_address;
    cmd_tx_data.data[2] = networkImformation.following_address;
    packCMDFrame(&cmd_tx_data);
    sendData(array2ByteArray());

    networkImformation.prev_sending_what = cmd_tx_data.mode;
    networkImformation.device_state = DEVICE_STATE_RECEVING;
}
