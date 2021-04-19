/*
* Copyright I3D Robotics Ltd, 2020
* Author: Ben Knight (bknight@i3drobotics.com)
*/

#include "abstractarduinocoms.h"

AbstractArduinoComs::AbstractArduinoComs(QObject *parent) : QObject(parent)
{
    m_serialPort = new QSerialPort();
    m_timer = new QTimer(this);
}


std::vector<QSerialPortInfo> AbstractArduinoComs::getSerialDevices(){
    std::vector<QSerialPortInfo> deviceList;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        deviceList.push_back(info);
        qDebug() << "Name : " << info.portName();
        qDebug() << "Description : " << info.description();
        qDebug() << "Manufacturer: " << info.manufacturer();
    }
    return deviceList;
}

bool AbstractArduinoComs::open(QSerialPortInfo serial_port_info,int baudrate){
    m_baudrate = baudrate;
    m_serialPortInfo = serial_port_info;
    m_timer->setSingleShot(true);
    m_serialPort->setPort(serial_port_info);
    m_serialPort->setBaudRate(baudrate);
    connected = m_serialPort->open(QIODevice::ReadWrite);

    if (connected){
        connect(m_serialPort, &QSerialPort::bytesWritten,
                this, &AbstractArduinoComs::handleBytesWritten);
        connect(m_serialPort, &QSerialPort::errorOccurred,
                this, &AbstractArduinoComs::handleError);
        connect(m_timer, &QTimer::timeout, this, &AbstractArduinoComs::handleTimeout);
    }

    return connected;
}

void AbstractArduinoComs::handleBytesWritten(qint64 bytes)
{
    m_bytesWritten += bytes;
    if (m_bytesWritten == m_writeData.size()) {
        m_bytesWritten = 0;
        m_standardOutput << QObject::tr("Data successfully sent to port %1")
                            .arg(m_serialPort->portName()) << "\n";
    }
}

void AbstractArduinoComs::handleTimeout()
{
    m_standardOutput << QObject::tr("Operation timed out for port %1, error: %2")
                        .arg(m_serialPort->portName(),
                             m_serialPort->errorString())
                     << "\n";
}

void AbstractArduinoComs::handleError(QSerialPort::SerialPortError serialPortError)
{
    if (serialPortError == QSerialPort::WriteError) {
        m_standardOutput << QObject::tr("An I/O error occurred while writing"
                                        " the data to port %1, error: %2")
                            .arg(m_serialPort->portName(),
                                 m_serialPort->errorString())
                         << "\n";
    }
}

void AbstractArduinoComs::write(const QString &writeData)
{
    m_writeData = writeData.toUtf8();

    const qint64 bytesWritten = m_serialPort->write(m_writeData);

    if (bytesWritten == -1) {
        m_standardOutput << QObject::tr("Failed to write the data to port %1, error: %2")
                            .arg(m_serialPort->portName(),
                                 m_serialPort->errorString())
                         << "\n";
    } else if (bytesWritten != m_writeData.size()) {
        m_standardOutput << QObject::tr("Failed to write all the data to port %1, error: %2")
                            .arg(m_serialPort->portName(),
                                 m_serialPort->errorString())
                         << "\n";
    }

    m_timer->start(5000);
}

void AbstractArduinoComs::close(){
    connected = false;

    if (m_serialPort){
        disconnect(m_serialPort, &QSerialPort::bytesWritten,
                this, &AbstractArduinoComs::handleBytesWritten);
        disconnect(m_serialPort, &QSerialPort::errorOccurred,
                this, &AbstractArduinoComs::handleError);
        disconnect(m_timer, &QTimer::timeout, this, &AbstractArduinoComs::handleTimeout);

        if (m_serialPort->isOpen()){
            m_serialPort->close();
            // TODO fix error:
            /*  ASSERT failure in QCoreApplication::sendEvent:
                "Cannot send events to objects owned by a different thread.
                Current thread 0x0x15363d27ff0. Receiver '' (of type 'QSerialPort') was
                created in thread 0x0x15354d90f50" */
        }
    }
}
