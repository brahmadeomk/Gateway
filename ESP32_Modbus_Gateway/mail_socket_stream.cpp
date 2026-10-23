#include "mail_socket_stream.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include <lwip/netdb.h>
#include <lwip/sockets.h>

namespace gwmail {
namespace {

// A mail server that has not answered in ten seconds is not going to. The alert
// queue is better served by failing and retrying on the backoff than by holding
// the task.
const uint32_t kSocketTimeoutMs = 10000;

}  // namespace

SocketStream::SocketStream()
    : socket_(-1), stage_(kStreamIdle), address_len_(0) {}

SocketStream::~SocketStream() { Close(); }

bool SocketStream::Open(const char* host, uint16_t port) {
    Close();
    stage_ = kStreamDns;
    address_len_ = 0;

    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    char port_text[8];
    snprintf(port_text, sizeof(port_text), "%u", static_cast<unsigned>(port));

    struct addrinfo* result = NULL;
    if (getaddrinfo(host, port_text, &hints, &result) != 0 || result == NULL) {
        if (result != NULL) {
            freeaddrinfo(result);
        }
        return false;
    }
    if (result->ai_addrlen > sizeof(address_)) {
        freeaddrinfo(result);
        return false;
    }
    memcpy(address_, result->ai_addr, result->ai_addrlen);
    address_len_ = result->ai_addrlen;
    freeaddrinfo(result);

    stage_ = kStreamConnect;
    socket_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (socket_ < 0) {
        return false;
    }

    // Both directions bounded, so a server that accepts the connection and then
    // says nothing cannot hold the calling task indefinitely.
    struct timeval timeout;
    timeout.tv_sec = static_cast<long>(kSocketTimeoutMs / 1000);
    timeout.tv_usec = static_cast<long>((kSocketTimeoutMs % 1000) * 1000);
    setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(socket_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (connect(socket_, reinterpret_cast<struct sockaddr*>(address_),
                static_cast<socklen_t>(address_len_)) != 0) {
        close(socket_);
        socket_ = -1;
        return false;
    }

    stage_ = kStreamOpen;
    return true;
}

bool SocketStream::WriteAll(const char* data, size_t len) {
    if (socket_ < 0) {
        return false;
    }
    size_t sent = 0;
    while (sent < len) {
        const int n = static_cast<int>(send(socket_, &data[sent], len - sent, 0));
        if (n <= 0) {
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

int SocketStream::Read(uint8_t* out, size_t len) {
    if (socket_ < 0) {
        return -1;
    }
    return static_cast<int>(recv(socket_, out, len, 0));
}

void SocketStream::Close() {
    if (socket_ >= 0) {
        close(socket_);
        socket_ = -1;
    }
    stage_ = kStreamIdle;
}

}  // namespace gwmail
