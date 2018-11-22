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

#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <iostream>
#include <vector>
#include <map>
#include <queue>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <mutex>
#include <memory>

class stampedMsg
{
public:
        stampedMsg(): msg("") {};
        // int socketfd;
        std::string msg;
        struct timeval tv;
        // bool removeUponReading;
};

class TCPServer
{
public:
        TCPServer();
        ~TCPServer();
        // std::vector<int> sockfd_vec;

        // pthread_t serverThread;
        // char msg[ MAXPACKETSIZE ];

        // static string Message;

        void setup(int port);
        void receive();
        std::vector<stampedMsg> getMessage();
        void Send(int, std::string);
        // void Send(string msg);
        void detach();
        void clean();

private:
        int sockfd_;
        struct sockaddr_in serverAddress_;
        struct sockaddr_in clientAddress_;
        void Task(int newsocketfd);
        // std::queue<stampedMsg> msg_queue_;
        // std::mutex msg_queue_mutex_;
        std::map<int, stampedMsg> msg_;
        std::map<int, std::unique_ptr<std::mutex> > msg_mutex_;
};

#endif
