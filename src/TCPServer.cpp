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

#include <drhmp_local_planner/tcp_server.h>

#include <sys/time.h> /* gethrtime(), gettimeofday() */

#include <mutex>
#include <thread>
#include <iostream>
#include <vector>
#include <functional>
#include <chrono>
#include <string>

// #define bthread boost::thread
#define MAXPACKETSIZE 4096
#define COMM_DEBUG_ENABLED (0)

#include <unistd.h>
#include <syscall.h>
#include <cstring>
void set_thread_params(int prio, int affinity, const std::string &name)
{
        pthread_t thId = pthread_self();
        sched_param sch_params;
        sch_params.sched_priority = prio;
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(affinity, &mask);
        if (pthread_setschedparam(thId, SCHED_FIFO, &sch_params))
        {
                std::cout << "Failed to set Thread scheduling of " << name << " ("
                        << (pid_t) syscall(SYS_gettid) << "), error "
                        << std::strerror(errno) << std::endl;
        }
        else
        {
                std::cout << "Thread set to priority of " << name << " ("
                        << (pid_t) syscall(SYS_gettid) << "), value " << prio
                        << std::endl;
        }
        if (pthread_setaffinity_np(thId, sizeof(cpu_set_t), &mask))
        {
                std::cout << "Failed to set Thread affinity of " << name << " ("
                        << (pid_t) syscall(SYS_gettid) << "), error "
                        << std::strerror(errno) << std::endl;
        }
        else
        {
                std::cout << "Thread set to affinity of " << name << " ("
                        << (pid_t) syscall(SYS_gettid) << "), value " << affinity
                        << std::endl;
        }
}

TCPServer::TCPServer()
{
        clean();
}

TCPServer::~TCPServer()
{
        detach();
}

void TCPServer::Task(int newsockfd)
{
        // set_thread_params(10, 1, "Server receive");
        if ( msg_.find(newsockfd) == msg_.end() )
        {
                msg_mutex_.emplace( newsockfd, std::make_unique<std::mutex>() );
                // std::mutex m;
                // msg_mutex_[newsockfd] = m;
                // msg_.insert(std::make_pair( newsockfd, stampedMsg() ));
                msg_.emplace(newsockfd, stampedMsg());
                // msg_[newsockfd] = stampedMsg();
                // [newsockfd] = std::mutex();
        }
        else
        {
                msg_[newsockfd].msg = "";
        }

        // cout << "[Task] newsockfd: " << newsockfd << endl;
        char msg[MAXPACKETSIZE];
        // pthread_detach(pthread_self());
        while(1)
        {
                int n;
                n=recv(newsockfd, msg, MAXPACKETSIZE, 0);
                if(n==0)
                {
                        close(newsockfd);
                        break;
                }

                msg[n] = 0;
                //send(newsockfd,msg,n,0);
                #if (COMM_DEBUG_ENABLED)
                std::cout <<  "\033[0;95m" << "#####\nrecv returned:" << "\033[0m" << std::endl;
                std::cout <<  "\033[0;95m" << std::string(msg) << "\033[0m" << std::endl;
                #endif

                stampedMsg Msg;
                Msg.msg = std::string(msg);
                gettimeofday(&Msg.tv, NULL);

                #if (COMM_DEBUG_ENABLED)
                std::cout <<  "\033[0;95m" << "tstamp: " << Msg.tv.tv_sec << ".";

                char prev = std::cout.fill('0');
                std::cout.width(6);
                std::cout << Msg.tv.tv_usec;
                std::cout.fill(prev);

                std::cout << "\n#####\033[0m" << std::endl;
                #endif

                {
                        // std::scoped_lock l(*(msg_mutex_[newsockfd])); // C++17
                        std::lock_guard<std::mutex> l(*(msg_mutex_[newsockfd]));
                        msg_[newsockfd].msg += Msg.msg;
                        msg_[newsockfd].tv = Msg.tv;
                        // msg_queue_.push(Msg);
                }
        }
}

void TCPServer::setup(int port)
{
        sockfd_ = socket(AF_INET,SOCK_STREAM, 0);
        memset(&serverAddress_, 0, sizeof(serverAddress_));
        serverAddress_.sin_family = AF_INET;
        serverAddress_.sin_addr.s_addr = htonl(INADDR_ANY);
        serverAddress_.sin_port = htons(port);
        bind(sockfd_, (struct sockaddr *)&serverAddress_, sizeof(serverAddress_));
        listen(sockfd_, 5);
}

void TCPServer::receive()
{
        // std::string clAddr;
        while(1)
        {
                socklen_t sosize  = sizeof(clientAddress_);
                int newsockfd = accept(sockfd_,(struct sockaddr*)&clientAddress_,&sosize);
                // cout << "Server listening on " << listen_port_ << " accepted connection from " <<
                                // inet_ntoa(clientAddress_.sin_addr) << "@" << ntohs(serverAddress_.sin_port) << endl << "newsockfd: " << newsockfd << endl;
                // clAddr = inet_ntoa(clientAddress_.sin_addr);
                std::thread aux(&TCPServer::Task, this, newsockfd);
                aux.detach();
        }
        // return clAddr;
}

std::vector<stampedMsg> TCPServer::getMessage()
{
        // return Message;
        // std::scoped_lock l(msg_queue_mutex_);
        std::vector<stampedMsg> msgs;
        for(auto &kv : msg_)
        {
                // std::scoped_lock l(*(msg_mutex_[kv.first])); // C++17
                std::lock_guard<std::mutex> l(*(msg_mutex_[kv.first]));
                if (kv.second.msg != "")
                {
                        msgs.push_back(kv.second);
                        kv.second.msg = "";
                        kv.second.tv.tv_sec = 0;
                        kv.second.tv.tv_usec = 0;
                }
        }
        return msgs;

        // std::scoped_lock l(*(msg_mutex_));
        // if (msg_.msg == "")
        // {
        //         stampedMsg empty_msg;
        //         empty_msg.msg = "";
        //         empty_msg.socketfd = 0;
        //         empty_msg.tv.tv_sec = 0;
        //         empty_msg.tv.tv_usec = 0;
        //         return empty_msg;
        // }
        //
        // // stampedMsg ret = msg_queue_.front();
        // // msg_queue_.pop();
        // stampedMsg msg(msg_);
        // msg_.msg = "";
        // msg_.socketfd = 0;
        // msg_.tv.tv_sec = 0;
        // msg_.tv.tv_usec = 0;
        // return msg;
}

void TCPServer::Send(int clsockfd, std::string msg)
{
        send(clsockfd, msg.c_str(),msg.length(),0);
}

void TCPServer::clean()
{
        // std::scoped_lock l(msg_queue_mutex_);
        // std::queue<stampedMsg>().swap(msg_queue_);

        for(auto &kv : msg_)
        {
                // std::scoped_lock l(*(msg_mutex_[kv.first])); // C++17
                std::lock_guard<std::mutex> l(*(msg_mutex_[kv.first]));
                kv.second.msg = "";
                kv.second.tv.tv_sec = 0;
                kv.second.tv.tv_usec = 0;
        }
        // Message = "";
        // memset(msg, 0, MAXPACKETSIZE);
}

void TCPServer::detach()
{
        close(sockfd_);
        sockfd_ = -1;
        clean();
        // std::scoped_lock l(msg_queue_mutex_);
        // while(!msg_queue_.empty())
        // {
        //         close(msg_queue_.front().socketfd);
        //         msg_queue_.pop();
        // }
}
