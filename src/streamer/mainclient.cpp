#include <winsock2.h>
#include <ws2tcpip.h>
#include <iostream>
#include <string>
#include <thread>

#include "base64.h"
#include "sha1.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "convertimage.h"

#define IMG_WIDTH 1920
#define IMG_HEIGHT 1080
 
using namespace std;
 
#pragma comment (lib, "Ws2_32.lib")

#define DEFAULT_BUFLEN 500000
//#define IP_ADDRESS "192.168.56.1"
//#define DEFAULT_PORT "3504"
#define IP_ADDRESS "127.0.0.1"
#define DEFAULT_PORT "8000"

struct client_type
{
    SOCKET socket;
    int id;
    char received_message[DEFAULT_BUFLEN];
};

cv::Mat str2Mat(std::string encoded){
    ImagemConverter conv = ImagemConverter();
    cv::Mat img = conv.str2mat(encoded);
    return img;
}

string mat2Str(const cv::Mat& image){
    ImagemConverter conv = ImagemConverter();
    std::string encoded = conv.mat2str(image);
    return encoded;
}

int process_client(client_type &new_client);
int main();
 
int process_client(client_type &new_client)
{
    while (1)
    {
        memset(new_client.received_message, 0, DEFAULT_BUFLEN);
 
        if (new_client.socket != 0)
        {
            int iResult = recv(new_client.socket, new_client.received_message, DEFAULT_BUFLEN, 0);
 
            if (iResult != SOCKET_ERROR){
                if (new_client.id == 1){
                    std::string msg = new_client.received_message;
                    if (!msg.empty() && msg[msg.length()-1] == '\n') {
                        msg.erase(msg.length()-1);
                    }
                    cv::Mat image = str2Mat(msg);
                    if (image.size().width > 0 && image.size().height > 0){
                        cv::imshow("receive", image);
                    } else {
                        cout << "Image is empty" << endl;
                        //cout << new_client.received_message << endl;
                    }
                    cv::waitKey(1);
                    //cout << new_client.received_message << endl;
                } else {
                    //cout << new_client.received_message << endl;
                }
                //cout << new_client.received_message << endl;
            }
            else
            {
                cout << "recv() failed: " << WSAGetLastError() << endl;
                break;
            }
        }
    }
 
    if (WSAGetLastError() == WSAECONNRESET)
        cout << "The server has disconnected" << endl;
 
    return 0;
}
 
int main()
{
    WSAData wsa_data;
    struct addrinfo *result = NULL, *ptr = NULL, hints;
    string sent_message = "";
    client_type client = { INVALID_SOCKET, -1, "" };
    int iResult = 0;
    string message;
 
    cout << "Starting Client...\n";
 
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (iResult != 0) {
        cout << "WSAStartup() failed with error: " << iResult << endl;
        return 1;
    }
 
    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
 
    cout << "Connecting...\n";
 
    // Resolve the server address and port
    iResult = getaddrinfo(static_cast<LPCTSTR>(IP_ADDRESS), DEFAULT_PORT, &hints, &result);
    if (iResult != 0) {
        cout << "getaddrinfo() failed with error: " << iResult << endl;
        WSACleanup();
        system("pause");
        return 1;
    }
 
    // Attempt to connect to an address until one succeeds
    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
 
        // Create a SOCKET for connecting to server
        client.socket = socket(ptr->ai_family, ptr->ai_socktype,
            ptr->ai_protocol);
        if (client.socket == INVALID_SOCKET) {
            cout << "socket() failed with error: " << WSAGetLastError() << endl;
            WSACleanup();
            system("pause");
            return 1;
        }
 
        // Connect to server.
        iResult = connect(client.socket, ptr->ai_addr, (int)ptr->ai_addrlen);
        if (iResult == SOCKET_ERROR) {
            closesocket(client.socket);
            client.socket = INVALID_SOCKET;
            continue;
        }
        break;
    }
 
    freeaddrinfo(result);
 
    if (client.socket == INVALID_SOCKET) {
        cout << "Unable to connect to server!" << endl;
        WSACleanup();
        system("pause");
        return 1;
    }
 
    cout << "Successfully Connected" << endl;
 
    //Obtain id from server for this client;
    recv(client.socket, client.received_message, DEFAULT_BUFLEN, 0);
    message = client.received_message;
 
    if (message != "Server is full")
    {
        client.id = atoi(client.received_message);
 
        thread my_thread(process_client, client);

        cv::VideoCapture capture("D:\\HOME\\OneDrive - i3d Robotics Ltd\\Stereo Theatre\\vids\\Clip.mp4");
        cv::Mat image;
 
        while (1)
        {
            if (client.id == 0){
                bool ret = capture.read(image);
                if (ret){
                    cv::resize(image, image, cv::Size(IMG_WIDTH, IMG_HEIGHT));
                    image.convertTo(image,CV_8UC1);
                    cv::imshow("send",image);
                    sent_message = mat2Str(image);
                    //cout << sent_message << std:endl;
                    std::cout << "Sending message of size: " << sent_message.size() << std::endl;
                    cv::waitKey(10);
                } else {
                    capture.set(cv::CAP_PROP_POS_FRAMES, 0);
                    cout << "Restarting video" << endl;
                }
            } else {
                getline(cin, sent_message);
                Sleep(100);
            }

            iResult = send(client.socket, sent_message.c_str(), strlen(sent_message.c_str()), 0);
 
            if (iResult <= 0)
            {
                cout << "send() failed: " << WSAGetLastError() << endl;
                break;
            }
        }
 
        //Shutdown the connection since no more data will be sent
        my_thread.detach();
    }
    else
        cout << client.received_message << endl;
 
    cout << "Shutting down socket..." << endl;
    iResult = shutdown(client.socket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        cout << "shutdown() failed with error: " << WSAGetLastError() << endl;
        closesocket(client.socket);
        WSACleanup();
        system("pause");
        return 1;
    }
 
    closesocket(client.socket);
    WSACleanup();
    //system("pause");
    return 0;
}