// A plain TCP socket as a ByteStream: the Wi-Fi uplink for SMTP.
//
// lwIP sockets rather than WiFiClient, because the SMTP client drives mbedtls
// itself - see platform/smtp_client.h for why STARTTLS makes that necessary.

#ifndef GWMAIL_SOCKET_STREAM_H
#define GWMAIL_SOCKET_STREAM_H

#include "mail_byte_stream.h"

namespace gwmail {

class SocketStream : public ByteStream {
  public:
    SocketStream();
    ~SocketStream();

    bool Open(const char* host, uint16_t port);
    bool WriteAll(const char* data, size_t len);
    int Read(uint8_t* out, size_t len);
    void Close();
    bool connected() const { return socket_ >= 0; }

    StreamStage stage() const { return stage_; }

    // The raw descriptor, which mbedtls needs for its network context.
    int fd() const { return socket_; }

  private:
    int socket_;
    StreamStage stage_;
    uint8_t address_[32];
    size_t address_len_;
};

}  // namespace gwmail

#endif  // GWMAIL_SOCKET_STREAM_H
