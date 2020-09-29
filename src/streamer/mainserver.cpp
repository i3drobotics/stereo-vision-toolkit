#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <thread>
#include <vector>
 
#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 500000
//#define IP_ADDRESS "192.168.56.1"
//#define DEFAULT_PORT "3504"
#define IP_ADDRESS "127.0.0.1"
#define DEFAULT_PORT "8000"
 
struct client_type
{
    int id;
    SOCKET socket;
};
 
const char OPTION_VALUE = 1;
const int MAX_CLIENTS = 5;
 
//Function Prototypes
int process_client(client_type &new_client, std::vector<client_type> &client_array, std::thread &thread);
int main();
 
int process_client(client_type &new_client, std::vector<client_type> &client_array, std::thread &thread)
{
    std::string msg = "";
    char tempmsg[DEFAULT_BUFLEN] = "";
 
    //Session
    while (1)
    {
        memset(tempmsg, 0, DEFAULT_BUFLEN);
 
        if (new_client.socket != 0)
        {
            int iResult = recv(new_client.socket, tempmsg, DEFAULT_BUFLEN, 0);

            std::string endlch = "\n";
 
            if (iResult != SOCKET_ERROR)
            {
                if (strcmp("", tempmsg))
                    //msg = "Client #" + std::to_string(new_client.id) + ": " + tempmsg;
                    msg = tempmsg + endlch;
                    //msg = tempmsg;
 
                //std::cout << msg.c_str() << std::endl;
 
                //Broadcast that message to the other clients
                for (int i = 0; i < MAX_CLIENTS; i++)
                {
                    if (client_array[i].socket != INVALID_SOCKET)
                        if (new_client.id != i)
                            iResult = send(client_array[i].socket, msg.c_str(), strlen(msg.c_str()), 0);
                }
            }
            else
            {
                msg = "Client #" + std::to_string(new_client.id) + " Disconnected";
 
                std::cout << msg << std::endl;
 
                closesocket(new_client.socket);
                closesocket(client_array[new_client.id].socket);
                client_array[new_client.id].socket = INVALID_SOCKET;
 
                //Broadcast the disconnection message to the other clients
                for (int i = 0; i < MAX_CLIENTS; i++)
                {
                    if (client_array[i].socket != INVALID_SOCKET)
                        iResult = send(client_array[i].socket, msg.c_str(), strlen(msg.c_str()), 0);
                }
 
                break;
            }
        }
    } //end while
 
    thread.detach();
 
    return 0;
}
 
int main()
{
    WSADATA wsaData;
    struct addrinfo hints;
    struct addrinfo *server = NULL;
    SOCKET server_socket = INVALID_SOCKET;
    std::string msg = "";
    std::vector<client_type> client(MAX_CLIENTS);
    int num_clients = 0;
    int temp_id = -1;
    std::thread my_thread[MAX_CLIENTS];
 
    //Initialize Winsock
    std::cout << "Intializing Winsock..." << std::endl;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
 
    //Setup hints
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;
 
    //Setup Server
    std::cout << "Setting up server..." << std::endl;
    getaddrinfo(static_cast<LPCTSTR>(IP_ADDRESS), DEFAULT_PORT, &hints, &server);
 
    //Create a listening socket for connecting to server
    std::cout << "Creating server socket..." << std::endl;
    server_socket = socket(server->ai_family, server->ai_socktype, server->ai_protocol);
 
    //Setup socket options
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &OPTION_VALUE, sizeof(int)); //Make it possible to re-bind to a port that was used within the last 2 minutes
    setsockopt(server_socket, IPPROTO_TCP, TCP_NODELAY, &OPTION_VALUE, sizeof(int)); //Used for interactive programs
 
    //Assign an address to the server socket.
    std::cout << "Binding socket..." << std::endl;
    bind(server_socket, server->ai_addr, (int)server->ai_addrlen);
 
    //Listen for incoming connections.
    std::cout << "Listening..." << std::endl;
    listen(server_socket, SOMAXCONN);
 
    //Initialize the client list
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        client[i] = { -1, INVALID_SOCKET };
    }
 
    while (1)
    {
 
        SOCKET incoming = INVALID_SOCKET;
        incoming = accept(server_socket, NULL, NULL);
 
        if (incoming == INVALID_SOCKET) continue;
 
        //Reset the number of clients
        num_clients = -1;
 
        //Create a temporary id for the next client
        temp_id = -1;
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            if (client[i].socket == INVALID_SOCKET && temp_id == -1)
            {
                client[i].socket = incoming;
                client[i].id = i;
                temp_id = i;
            }
 
            if (client[i].socket != INVALID_SOCKET)
                num_clients++;
 
            //std::cout << client[i].socket << std::endl;
        }
 
        if (temp_id != -1)
        {
            //Send the id to that client
            std::cout << "Client #" << client[temp_id].id << " Accepted" << std::endl;
            msg = std::to_string(client[temp_id].id);
            send(client[temp_id].socket, msg.c_str(), strlen(msg.c_str()), 0);
 
            //Create a thread process for that client
            my_thread[temp_id] = std::thread(process_client, std::ref(client[temp_id]), std::ref(client), std::ref(my_thread[temp_id]));
        }
        else
        {
            msg = "Server is full";
            send(incoming, msg.c_str(), strlen(msg.c_str()), 0);
            std::cout << msg << std::endl;
        }
    } //end while
 
 
    //Close listening socket
    closesocket(server_socket);
 
    //Close client socket
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        my_thread[i].detach();
        closesocket(client[i].socket);
    }
 
    //Clean up Winsock
    WSACleanup();
    std::cout << "Program has ended successfully" << std::endl;
 
    system("pause");
    return 0;
}