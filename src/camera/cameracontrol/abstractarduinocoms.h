#ifndef ABSTRACTARDUINOCOMS_H
#define ABSTRACTARDUINOCOMS_H

#include <QObject>
#include <QDebug>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QTimer>

//!  Arduino comms
/*!
  Commuicate with arduino via serial (QSerialPort)
*/
class AbstractArduinoComs : public QObject
{
    Q_OBJECT
public:
    explicit AbstractArduinoComs(QObject *parent = nullptr);

    std::vector<QSerialPortInfo> getSerialDevices();

    bool open(QSerialPortInfo serial_port_info, int baudrate);

    void write(const QString &writeData);

    void close();

    bool isConnected(){return connected;};

private slots:
    void handleBytesWritten(qint64 bytes);
    void handleTimeout();
    void handleError(QSerialPort::SerialPortError error);

private:
    bool connected = false;
    QSerialPort *m_serialPort = nullptr;
    QByteArray m_writeData;
    QTextStream m_standardOutput;
    qint64 m_bytesWritten = 0;
    QTimer *m_timer;

protected:
    QSerialPortInfo m_serialPortInfo;
    int m_baudrate = 115200;

signals:

};

#endif // ABSTRACTARDUINOCOMS_H
