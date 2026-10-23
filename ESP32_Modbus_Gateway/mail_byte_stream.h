// A bidirectional byte stream, so the SMTP client does not know or care which
// uplink it is running over.
//
// This exists to avoid the obvious alternative, which is a second SMTP
// implementation for cellular. That alternative is how devices end up with a
// weaker security posture on the fallback path: the Wi-Fi path verifies
// certificates and the cellular path quietly does not, because the modem's
// own AT-command TLS is harder to drive and nobody revisits it.
//
// With one stream interface, core/smtp_session.* drives the same conversation,
// mbedtls performs the same handshake with the same verification, and
// core/mime_writer.* produces the same bytes, whichever transport is
// underneath. The only thing that changes is where the bytes go.
//
// Implementations:
//   SocketStream  - lwIP TCP socket over Wi-Fi (platform/socket_stream.*)
//   ModemStream   - AT-command socket on the SIM7600G-H
//                   (platform/cellular_modem.*)

#ifndef GWMAIL_BYTE_STREAM_H
#define GWMAIL_BYTE_STREAM_H

#include <stddef.h>
#include <stdint.h>

namespace gwmail {

// Where an Open() attempt stopped. DNS resolution and the TCP connect are
// reported separately - "cannot resolve the server name" and
// "the server refused the connection" send a technician to different places -
// so the distinction has to survive the transport abstraction.
enum StreamStage {
    kStreamIdle = 0,
    kStreamDns,
    kStreamConnect,
    kStreamOpen,
};

class ByteStream {
  public:
    virtual ~ByteStream() {}

    // Open a connection. Returns false on any failure; `Stage()` on the
    // concrete class says where it got to.
    virtual bool Open(const char* host, uint16_t port) = 0;

    // Write all of `len` bytes. Returns false if the connection failed
    // part-way - a partial write is never reported as success, because a
    // truncated SMTP command is a different command.
    virtual bool WriteAll(const char* data, size_t len) = 0;

    // Read up to `len` bytes. Returns the count, 0 on timeout with the
    // connection still open, or negative on error or close.
    virtual int Read(uint8_t* out, size_t len) = 0;

    virtual void Close() = 0;
    virtual bool connected() const = 0;

    // How far Open() got. Read after a failed Open().
    virtual StreamStage stage() const = 0;
};

}  // namespace gwmail

#endif  // GWMAIL_BYTE_STREAM_H
