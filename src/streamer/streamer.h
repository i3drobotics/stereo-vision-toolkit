#ifndef STREAMER_H
#define STREAMER_H

#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <thread>
#include <vector>

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QCoreApplication>
#include <QtConcurrent>

#pragma comment (lib, "Ws2_32.lib")

#define STREAMER_MAX_CLIENTS 5
#define STREAMER_MAX_BUFFER_LENGTH 500000

class Streamer  : public QObject {
    Q_OBJECT
public:
    struct client_type
    {
        int id;
        SOCKET socket;
    };

    explicit Streamer(std::string ip="127.0.0.1", std::string port="8000",QObject *parent = nullptr);
    void assignThread(QThread* thread);
    void setIP(std::string ip){ip_ = ip;};
    void setPort(std::string port){port_ = port;};

    ~Streamer();

public slots:
    void startServer();
    void stopServer();

private slots:
    void serverCycleThreaded();
    void serverCycle();

private:
    std::string ip_;
    std::string port_;

    char option_value_ = 1;
    int num_clients_ = 0;
    int temp_id_ = -1;

    QFuture<void> qfuture_serverCycle;

    SOCKET server_socket_;
    std::vector<client_type> client_;
    std::thread clientThread_[STREAMER_MAX_CLIENTS];
    QThread* thread_;

    static int process_client(client_type &new_client, std::vector<client_type> &client_array, std::thread &thread);

signals:
    void finished();
    void serverCycleComplete();
};

#endif // STREAMER_H
