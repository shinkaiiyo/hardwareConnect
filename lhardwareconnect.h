#ifndef LHARDWARECONNECT_H
#define LHARDWARECONNECT_H

#include <QObject>
#include <QWidget>

class SerialPort;
class QThread;
class LhardwareConnect : public QObject
{
    Q_OBJECT

public:
    explicit LhardwareConnect(QWidget *parent = 0);
    ~LhardwareConnect();
    void createConnection();
    void close();

public slots:
    void onRecvedData(QByteArray data);
    void onRecvedErrorData(QByteArray data);
    void onSendData(QByteArray data);
    void onConnected();
    void onDisconnected();

private:
    SerialPort*             m_port;
    QThread*                m_threadForSerial;

signals:
    void signalConnected();
    void signalDisConnected();
    void signalSensorInfo(QByteArray data);
    void signalSendData(QByteArray data);
};

#endif // LHARDWARECONNECT_H
