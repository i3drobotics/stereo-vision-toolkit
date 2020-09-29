#include "streamer.h"

Streamer::Streamer(std::string ip, std::string port, QObject *parent) : QObject(parent), ip_(ip), port_(port)
{
    client_ = std::vector<client_type>(STREAMER_MAX_CLIENTS);
}

void Streamer::assignThread(QThread *thread){
    qDebug() << "Moving streamer to thread";
    thread_ = thread;
    this->moveToThread(thread_);
    connect(this, SIGNAL(finished()), thread_, SLOT(quit()));
    //connect(this, SIGNAL(finished()), this, SLOT(deleteLater()));
    connect(thread_, SIGNAL(finished()), thread_, SLOT(deleteLater()));
    thread_->start();
    thread_->setPriority(QThread::LowestPriority);
}

void Streamer::startServer(){
    WSADATA wsaData;
    struct addrinfo hints;
    struct addrinfo *server = NULL;
    server_socket_ = INVALID_SOCKET;

    //Initialize Winsock
    qDebug() << "Intializing Winsock...";
    //std::cout << "Intializing Winsock..." << std::endl;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    //Setup hints
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    //Setup Server
    qDebug() << "Setting up server...";
    //std::cout << "Setting up server..." << std::endl;
    getaddrinfo(ip_.c_str(), port_.c_str(), &hints, &server);

    //Create a listening socket for connecting to server
    qDebug() << "Creating server socket...";
    //std::cout << "Creating server socket..." << std::endl;
    server_socket_ = socket(server->ai_family, server->ai_socktype, server->ai_protocol);

    //Setup socket options
    setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &option_value_, sizeof(int)); //Make it possible to re-bind to a port that was used within the last 2 minutes
    setsockopt(server_socket_, IPPROTO_TCP, TCP_NODELAY, &option_value_, sizeof(int)); //Used for interactive programs

    //Assign an address to the server socket.
    qDebug() << "Binding socket...";
    //std::cout << "Binding socket..." << std::endl;
    bind(server_socket_, server->ai_addr, (int)server->ai_addrlen);

    //Listen for incoming connections.
    qDebug() << "Listening...";
    //std::cout << "Listening..." << std::endl;
    listen(server_socket_, SOMAXCONN);

    //Initialize the client list
    for (int i = 0; i < STREAMER_MAX_CLIENTS; i++)
    {
        client_[i] = { -1, INVALID_SOCKET };
    }

    connect(this, SIGNAL(serverCycleComplete()), this, SLOT(serverCycleThreaded()));
    serverCycleThreaded();
}

void Streamer::serverCycleThreaded(){
    qfuture_serverCycle = QtConcurrent::run(this, &Streamer::serverCycle);
}

int Streamer::process_client(client_type &new_client, std::vector<client_type> &client_array, std::thread &thread)
{
    std::string msg = "";
    char tempmsg[STREAMER_MAX_BUFFER_LENGTH] = "";

    //Session
    while (1)
    {
        memset(tempmsg, 0, STREAMER_MAX_BUFFER_LENGTH);

        if (new_client.socket != 0)
        {
            int iResult = recv(new_client.socket, tempmsg, STREAMER_MAX_BUFFER_LENGTH, 0);

            std::string endlch = "\n";

            if (iResult != SOCKET_ERROR)
            {
                if (strcmp("", tempmsg))
                    //msg = "Client #" + std::to_string(new_client.id) + ": " + tempmsg;
                    msg = tempmsg + endlch;
                    //msg = tempmsg;

                //qDebug() << msg.c_str();
                //std::cout << msg.c_str() << std::endl;

                //Broadcast that message to the other clients
                for (int i = 0; i < STREAMER_MAX_CLIENTS; i++)
                {
                    if (client_array[i].socket != INVALID_SOCKET)
                        if (new_client.id != i)
                            iResult = send(client_array[i].socket, msg.c_str(), strlen(msg.c_str()), 0);
                }
            }
            else
            {
                msg = "Client #" + std::to_string(new_client.id) + " Disconnected";

                qDebug() << msg.c_str();
                //std::cout << msg << std::endl;

                closesocket(new_client.socket);
                closesocket(client_array[new_client.id].socket);
                client_array[new_client.id].socket = INVALID_SOCKET;

                //Broadcast the disconnection message to the other clients
                for (int i = 0; i < STREAMER_MAX_CLIENTS; i++)
                {
                    if (client_array[i].socket != INVALID_SOCKET)
                        iResult = send(client_array[i].socket, msg.c_str(), strlen(msg.c_str()), 0);
                }

                break;
            }
        }
    }

    thread.detach();

    return 0;
}

void Streamer::serverCycle(){
    std::string msg = "";
    SOCKET incoming = INVALID_SOCKET;
    incoming = accept(server_socket_, NULL, NULL);

    if (incoming == INVALID_SOCKET) return;

    //Reset the number of clients
    num_clients_ = -1;

    //Create a temporary id for the next client
    temp_id_ = -1;
    for (int i = 0; i < STREAMER_MAX_CLIENTS; i++)
    {
        if (client_[i].socket == INVALID_SOCKET && temp_id_ == -1)
        {
            client_[i].socket = incoming;
            client_[i].id = i;
            temp_id_ = i;
        }

        if (client_[i].socket != INVALID_SOCKET)
            num_clients_++;

        //std::cout << client[i].socket << std::endl;
    }

    if (temp_id_ != -1)
    {
        //Send the id to that client
        qDebug() << "Client #" << client_[temp_id_].id << " Accepted";
        //std::cout << "Client #" << client_[temp_id_].id << " Accepted" << std::endl;
        msg = std::to_string(client_[temp_id_].id);
        send(client_[temp_id_].socket, msg.c_str(), strlen(msg.c_str()), 0);

        //Create a thread process for that client
        clientThread_[temp_id_] = std::thread(&Streamer::process_client, std::ref(client_[temp_id_]), std::ref(client_), std::ref(clientThread_[temp_id_]));
    }
    else
    {
        msg = "Server is full";
        send(incoming, msg.c_str(), strlen(msg.c_str()), 0);
        qDebug() << msg.c_str();
        //std::cout << msg << std::endl;
    }

    emit serverCycleComplete();
}

void Streamer::stopServer(){
    disconnect(this, SIGNAL(serverCycleComplete()), this, SLOT(serverCycleThreaded()));

    //Close listening socket
    closesocket(server_socket_);

    //Close client socket
    for (int i = 0; i < STREAMER_MAX_CLIENTS; i++)
    {
        clientThread_[i].detach();
        closesocket(client_[i].socket);
    }

    //Clean up Winsock
    WSACleanup();
}

Streamer::~Streamer(){
    emit finished();
}
