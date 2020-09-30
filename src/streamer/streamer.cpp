#include "streamer.h"

Streamer::Streamer(std::string ip, std::string port, QObject *parent) : QObject(parent), ip_(ip), port_(port)
{
    client_ = std::vector<client_type>(STREAMER_MAX_CLIENTS);
    for (int i = 0; i < STREAMER_MAX_CLIENTS; i++)
    {
        clientThread_[i] = std::thread();
    }
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

Streamer::client_type Streamer::startClient(){
    WSAData wsa_data;
    struct addrinfo *result = NULL, *ptr = NULL, hints;
    std::string sent_message = "";
    client_type client = { -1, INVALID_SOCKET};
    int iResult = 0;
    std::string message;

    qDebug() << "Starting Client...";
    //std::cout << "Starting Client...\n";

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (iResult != 0) {
        qDebug() << "WSAStartup() failed with error: " << iResult;
        //std::cout << "WSAStartup() failed with error: " << iResult << std::endl;
        return client;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    qDebug() <<"Connecting..";
    //std::cout << "Connecting...\n";

    // Resolve the server address and port
    iResult = getaddrinfo(ip_.c_str(), port_.c_str(), &hints, &result);
    if (iResult != 0) {
        qDebug() << "getaddrinfo() failed with error: " << iResult;
        //std::cout << "getaddrinfo() failed with error: " << iResult << std::endl;
        return client;
    }

    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

        // Create a SOCKET for connecting to server
        client.socket = socket(ptr->ai_family, ptr->ai_socktype,
            ptr->ai_protocol);
        if (client.socket == INVALID_SOCKET) {
            qDebug() << "socket() failed with error: " << WSAGetLastError();
            //std::cout << "socket() failed with error: " << WSAGetLastError() << std::endl;
            return client;
        }

        // Connect to server.
        iResult = ::connect(client.socket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(client.socket);
            client.socket = INVALID_SOCKET;
            continue;
        }
        break;
    }

    freeaddrinfo(result);

    if (client.socket == INVALID_SOCKET) {
        qDebug() << "Unable to connect to server!";
        //std::cout << "Unable to connect to server!" << std::endl;
        return client;
    }

    qDebug() << "Successfully Connected";
    //std::cout << "Successfully Connected" << std::endl;

    //Obtain id from server for this client;
    char received_message[STREAMER_MAX_BUFFER_LENGTH];
    recv(client.socket, received_message, STREAMER_MAX_BUFFER_LENGTH, 0);
    message = received_message;

    if (message != "Server is full")
    {
        client.id = atoi(received_message);
        return client;
    } else {
        return client;
    }
}

void Streamer::clientSendImageThreaded(client_type id, cv::Mat image){
    if (qfuture_clientSendThreaded != nullptr){
        if (qfuture_clientSendThreaded->isFinished()){
            cv::Mat send_image = image;
            qfuture_clientSendThreaded = new QFuture<bool>(QtConcurrent::run(Streamer::clientSendImage, id, send_image));
        }
    } else {
        cv::Mat send_image = image;
        qfuture_clientSendThreaded = new QFuture<bool>(QtConcurrent::run(Streamer::clientSendImage, id, send_image));
    }
}

bool Streamer::clientSendMessage(client_type client, std::string message){
    int iResult = send(client.socket, message.c_str(), strlen(message.c_str()), 0);
    if (iResult <= 0)
    {
        qDebug() << "send() failed: " << WSAGetLastError();
        //std::cout << "send() failed: " << WSAGetLastError() << endl;
        return false;
    } else {
        return true;
    }
}

bool Streamer::clientSendImage(client_type client, cv::Mat image){
    cv::resize(image, image, cv::Size(TEST_IMG_WIDTH, TEST_IMG_HEIGHT));
    image.convertTo(image,CV_8UC1);
    std::string message = Image2String::mat2str(image,96);
    qDebug() << "Sending image message of size: " << message.size();
    return clientSendMessage(client,message);
}

bool Streamer::stopClient(client_type client){
    qDebug() << "Shutting down socket...";
    //std::cout << "Shutting down socket..." << endl;
    int iResult = shutdown(client.socket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        qDebug() << "shutdown() failed with error: " << WSAGetLastError();
        //std::cout << "shutdown() failed with error: " << WSAGetLastError() << endl;
        closesocket(client.socket);
    }
    closesocket(client.socket);
    return true;
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
    //serverCycle(); //do one non threaded cycle to inialise the server
    serverCycleThreaded();
}

void Streamer::serverCycleThreaded(){
    qfuture_serverCycle = new QFuture<void>(QtConcurrent::run(this, &Streamer::serverCycle));
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

    if (incoming != INVALID_SOCKET){
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
        /*
        if (clientThread_[i].joinable())
            clientThread_[i].detach();
        */
        closesocket(client_[i].socket);
    }

    //Clean up Winsock
    WSACleanup();
}

Streamer::~Streamer(){
    emit finished();
}
