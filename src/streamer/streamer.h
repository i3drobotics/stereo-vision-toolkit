#ifndef STREAMER_H
#define STREAMER_H

#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <thread>
#include <vector>
#include <cmath>

#include "image2string.h"

#include <QObject>
#include <QThread>
#include <QDebug>
#include <QCoreApplication>
#include <QtConcurrent>

#pragma comment (lib, "Ws2_32.lib")

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

    Streamer::client_type startClient();
    bool stopClient(Streamer::client_type id);

    static bool clientSendMessage(Streamer::client_type id, std::string msg);
    static bool clientSendUCharImage(Streamer::client_type client, cv::Mat image);
    static bool clientSendFloatImage(Streamer::client_type client, cv::Mat image);
    bool clientSendUCharImageThreaded(Streamer::client_type id, cv::Mat image);
    bool clientSendFloatImageThreaded(Streamer::client_type id, cv::Mat image);

private slots:
    void serverCycleThreaded();
    void serverCycle();

private:
    std::string ip_;
    std::string port_;

    bool sendingMessage = false;

    static const int max_clients_ = 5;
    static const size_t max_buffer_length_ = 65535;

    static const char eom_token_ = '\n';

    char option_value_ = 1;
    int num_clients_ = 0;

    QFuture<void> *qfuture_serverCycle = nullptr;
    std::vector<QFuture<bool>*> qfuture_clientSendThreaded;

    SOCKET server_socket_;
    std::vector<Streamer::client_type> client_;
    std::thread clientThread_[max_clients_];
    QThread* thread_;

    static int process_client(Streamer::client_type &new_client, std::vector<Streamer::client_type> &client_array, std::thread &thread);
    static bool clientSendMessagePacket(Streamer::client_type id, std::string msg);

signals:
    void finished();
    void serverCycleComplete();
};

#endif // STREAMER_H
