/*******************************************************************************
 * 2-Clause BSD License
 *
 * Copyright (c) 2018, iFollow SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: José MENDES FILHO
 ******************************************************************************/

#include <drhmp_local_planner/tcp_client.h>
#include <thread>
#include <chrono>

#define COMM_DEBUG_ENABLED (1)

TCPClient::TCPClient()
        : sock(-1)
        , address("")
        , port(0)
        , isSetup_(false)
{}

void TCPClient::async_setup(std::string address, int port)
{
        std::thread aux(&TCPClient::setup_loop, this, address, port);
        // bthread aux(&TCPClient::setup_loop, this, address, port);
        aux.detach();
}

void TCPClient::setup_loop(std::string address, int port)
{
        while (true)
        {
                if (!isSetup_)
                {
                        isSetup_ = setup(address, port);
                }
                // set waiting time between setups call?
                // FIXME should be a configurable parameter
                std::this_thread::sleep_for(std::chrono::milliseconds(60));
                // boost::this_thread::sleep_for(boost::chrono::milliseconds(60));
        }
}

bool TCPClient::setup(std::string address, int port)
{
        if(sock == -1)
        {
                sock = socket(AF_INET, SOCK_STREAM, 0);
                if (sock == -1)
                {
                        #if (COMM_DEBUG_ENABLED)
                        std::cout << "\033[1;95m" << "Could not create socket" << "\033[0m" << std::endl;
                        #endif
                }
        }
        if(inet_addr(address.c_str()) == -1)
        {
                struct hostent *he;
                struct in_addr **addr_list;
                if ( (he = gethostbyname( address.c_str() ) ) == NULL)
                {
                        herror("gethostbyname");
                        #if (COMM_DEBUG_ENABLED)
                        std::cout << "\033[1;95m" << "Failed to resolve hostname: " << address << "\033[0m" << std::endl;
                        #endif
                        return false;
                }
                addr_list = (struct in_addr **) he->h_addr_list;
                for(int i = 0; addr_list[i] != NULL; i++)
                {
                        server.sin_addr = *addr_list[i];
                        break;
                }
        }
        else
        {
                server.sin_addr.s_addr = inet_addr( address.c_str() );
        }
        server.sin_family = AF_INET;
        server.sin_port = htons( port );
        if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0)
        {
                // std::string err("connect to " + address + ":" + std::to_string(port) + " failed. Error");
                #if (COMM_DEBUG_ENABLED)
                std::cout << "\033[1;95m" << "Connection to " << address << ":" << port << " failed" << "\033[0m" << std::endl;
                std::string err("\tError");
                perror(err.c_str());
                #endif
                return false;
        }
        return true;
}

bool TCPClient::Send(std::string data)
{
        if(sock != -1)
        {
                if( send(sock, data.c_str(), strlen( data.c_str() ), 0) < 0)
                {
                        #if (COMM_DEBUG_ENABLED)
                        std::cout << "\033[1;95m" << "Send failed!" << "\033[0m" << std::endl;
                        #endif

                        isSetup_ = false;
                        exit();
                        return false;
                }
        }
        else
        {
                isSetup_ = false;
                exit();
                return false;
        }
        return true;
}

std::string TCPClient::receive(int size)
{
        char buffer[size];
        memset(&buffer[0], 0, sizeof(buffer));

        std::string reply;
        if( recv(sock, buffer, size, 0) < 0)
        {
                #if (COMM_DEBUG_ENABLED)
                std::cout << "\033[1;95m" << "Receive failed!" << "\033[0m" << std::endl;
                #endif

                isSetup_ = false;
                exit();
                return nullptr;
        }
        buffer[size-1]='\0';
        reply = buffer;
        return reply;
}

std::string TCPClient::read()
{
        char buffer[1] = {};
        std::string reply;
        while (buffer[0] != '\n') {
                if( recv(sock, buffer, sizeof(buffer), 0) < 0)
                {
                        #if (COMM_DEBUG_ENABLED)
                        std::cout << "receive failed!" << std::endl;
                        #endif

                        isSetup_ = false;
                        exit();
                        return nullptr;
                }
                reply += buffer[0];
        }
        return reply;
}

void TCPClient::exit()
{
        close( sock );
        sock = -1;
}
