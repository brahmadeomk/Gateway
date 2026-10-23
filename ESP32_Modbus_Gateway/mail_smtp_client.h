// SMTP transport: sockets, TLS, timeouts.
//
// The conversation itself is in core/smtp_session.*, which knows no sockets and
// is tested against real reply sequences. This file is the part that cannot be
// tested on the host: name resolution, the TCP connect, the TLS handshake, and
// pushing the MIME stream out without ever holding it.
//
// Transport independence
// ----------------------
// The connection arrives as a ByteStream (platform/byte_stream.h), so the same
// client, the same conversation and the same TLS run over Wi-Fi or over the
// SIM7600G-H's AT socket. Writing a second SMTP path for cellular is how a
// device ends up verifying certificates on one uplink and not the other.
//
// Why raw sockets and mbedtls rather than WiFiClientSecure
// --------------------------------------------------------
// STARTTLS needs a connection that begins in plaintext, exchanges EHLO and
// STARTTLS, and only then negotiates TLS on the same socket. The Arduino
// WiFiClientSecure API has no way to upgrade an existing connection - it
// negotiates at connect() or not at all - so STARTTLS - the default mode,
// and what Office 365 and most providers use on port 587 - would be
// unimplementable on top of it. Driving mbedtls over
// an lwIP socket gives both modes from one code path.
//
// Certificate verification
// ------------------------
// Verification is REQUIRED and checked against the ESP-IDF Mozilla CA bundle,
// with the server hostname checked against the certificate. This device sends
// SMTP credentials to that server; accepting any certificate would mean any
// host that can answer for the address collects the password. (The inbound
// web UI being plain HTTP is a separate, LAN-only decision and says nothing
// about an outbound connection across a site network.)

#ifndef GWMAIL_SMTP_CLIENT_H
#define GWMAIL_SMTP_CLIENT_H

#include <stddef.h>
#include <stdint.h>

#include <mbedtls/ctr_drbg.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ssl.h>

#include "mail_config.h"
#include "mail_mime_writer.h"
#include "mail_smtp_session.h"
#include "mail_byte_stream.h"

namespace gwmail {

// Per-operation timeout. A mail server that has not answered a command in ten
// seconds is not going to; the alert queue is better served by failing and
// retrying on the backoff than by holding the task.
static const uint32_t kSmtpTimeoutMs = 10000;

// How long to keep waiting for one complete reply line before giving up.
//
// Longer than the per-read timeout on purpose. A socket read returns as soon
// as bytes arrive, but the modem transport POLLS - it asks the module whether
// anything has turned up and legitimately answers "not yet". Treating that as
// a failure would abandon the connection while the server was still composing
// its greeting, which is exactly what it did.
static const uint32_t kSmtpReplyDeadlineMs = 30000;

// Ceiling on the TLS handshake. Generous, because a handshake over cellular
// moves several kilobytes of certificate chain, but finite: the mail task runs
// without a watchdog, so a handshake that never completes would stop the device
// sending mail at all rather than failing one send.
static const uint32_t kSmtpHandshakeDeadlineMs = 45000;

struct SmtpResult {
    bool ok;
    SmtpStage stage;   // where it got to, or where it failed
    int code;          // server's reply code at the failure, 0 if not one
    int accepted;      // recipients the server took
    int rejected;
    size_t bytes_sent;

    SmtpResult()
        : ok(false),
          stage(kSmtpStageIdle),
          code(0),
          accepted(0),
          rejected(0),
          bytes_sent(0) {}
};

// Human-readable one-line summary for the portal and the Status Log. Never
// includes a credential - only the stage, the code and the counts.
void FormatSmtpResult(const SmtpResult& result, char* out, size_t out_len);

class SmtpClient {
  public:
    SmtpClient();
    ~SmtpClient();

    // Send one message over `stream`, blocking until it is accepted or fails.
    // Must be called from Core 0 - it can block for seconds, and Core 1 is
    // reserved for Modbus timing.
    //
    // The caller owns `stream` and is responsible for whatever the transport
    // needs before this is called - a registered modem, or a live Wi-Fi link.
    SmtpResult Send(const EmailConfig& config, const char* client_hostname,
                    const MimeMessage& message, ByteStream* stream);

  private:
    static int BioSend(void* ctx, const unsigned char* buf, size_t len);
    static int BioRecv(void* ctx, unsigned char* buf, size_t len);

    bool BeginTls(const char* host);
    void EndTls();
    void Close();

    bool WriteAll(const char* data, size_t len);
    bool ReadLine(char* out, size_t out_len);
    bool StreamBody(const MimeMessage& message, size_t* bytes_sent);

    ByteStream* stream_;
    bool tls_active_;
    bool tls_ready_;

    // Receive buffer. SMTP reply lines are short; 512 covers the longest EHLO
    // capability line any server sends in practice.
    char rx_[512];
    size_t rx_len_;
    size_t rx_pos_;

    mbedtls_ssl_context ssl_;
    mbedtls_ssl_config conf_;
    mbedtls_ctr_drbg_context drbg_;
    mbedtls_entropy_context entropy_;
};

}  // namespace gwmail

#endif  // GWMAIL_SMTP_CLIENT_H
