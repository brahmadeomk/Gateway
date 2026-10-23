#include "mail_smtp_client.h"

#include <Arduino.h>
#include <stdio.h>
#include <string.h>

#include <esp_crt_bundle.h>

namespace gwmail {
namespace {

// One MIME slice at a time. The message is never held: MimeWriter fills this,
// it goes to the socket, and it is refilled.
const size_t kStreamChunkBytes = 512;

}  // namespace

void FormatSmtpResult(const SmtpResult& result, char* out, size_t out_len) {
    if (out == NULL || out_len == 0) {
        return;
    }
    if (result.ok) {
        snprintf(out, out_len, "sent to %d recipient%s%s", result.accepted,
                 (result.accepted == 1) ? "" : "s",
                 (result.rejected > 0) ? " (some addresses rejected)" : "");
        return;
    }
    // Name the stage, and the server's code when there was one.
    // "SMTP failed" is what sends a technician away from site.
    if (result.code > 0) {
        snprintf(out, out_len, "failed at %s: server said %d",
                 SmtpStageText(result.stage), result.code);
    } else {
        snprintf(out, out_len, "failed at %s", SmtpStageText(result.stage));
    }
}

SmtpClient::SmtpClient()
    : stream_(NULL),
      tls_active_(false),
      tls_ready_(false),
      rx_len_(0),
      rx_pos_(0) {
    rx_[0] = '\0';
}

// mbedtls talks to whatever transport the caller supplied. These two callbacks
// are the entire difference between sending over Wi-Fi and sending over the
// modem - everything above them, including certificate verification, is shared.
int SmtpClient::BioSend(void* ctx, const unsigned char* buf, size_t len) {
    ByteStream* stream = static_cast<ByteStream*>(ctx);
    if (stream == NULL) {
        return -1;
    }
    if (!stream->WriteAll(reinterpret_cast<const char*>(buf), len)) {
        return -1;
    }
    return static_cast<int>(len);
}

int SmtpClient::BioRecv(void* ctx, unsigned char* buf, size_t len) {
    ByteStream* stream = static_cast<ByteStream*>(ctx);
    if (stream == NULL) {
        return -1;
    }
    const int n = stream->Read(buf, len);
    if (n == 0) {
        // The stream's own read already blocked for its timeout, so asking
        // mbedtls to retry costs nothing and keeps a slow link alive.
        return MBEDTLS_ERR_SSL_WANT_READ;
    }
    return n;
}

SmtpClient::~SmtpClient() { Close(); }

bool SmtpClient::BeginTls(const char* host) {
    mbedtls_ssl_init(&ssl_);
    mbedtls_ssl_config_init(&conf_);
    mbedtls_ctr_drbg_init(&drbg_);
    mbedtls_entropy_init(&entropy_);
    tls_ready_ = true;

    static const char kSeed[] = "gateway-smtp";
    if (mbedtls_ctr_drbg_seed(&drbg_, mbedtls_entropy_func, &entropy_,
                              reinterpret_cast<const unsigned char*>(kSeed),
                              sizeof(kSeed) - 1) != 0) {
        return false;
    }
    if (mbedtls_ssl_config_defaults(&conf_, MBEDTLS_SSL_IS_CLIENT,
                                    MBEDTLS_SSL_TRANSPORT_STREAM,
                                    MBEDTLS_SSL_PRESET_DEFAULT) != 0) {
        return false;
    }

    // Verification is required, against the CA bundle built into ESP-IDF. This
    // device is about to send an SMTP password over this connection; accepting
    // an unverified certificate would hand it to whatever answered.
    mbedtls_ssl_conf_authmode(&conf_, MBEDTLS_SSL_VERIFY_REQUIRED);
    if (esp_crt_bundle_attach(&conf_) != 0) {
        return false;
    }
    mbedtls_ssl_conf_rng(&conf_, mbedtls_ctr_drbg_random, &drbg_);

    if (mbedtls_ssl_setup(&ssl_, &conf_) != 0) {
        return false;
    }
    // Checks the name in the certificate, not just its signature chain. Without
    // this any certificate the bundle trusts would pass, for any host.
    if (mbedtls_ssl_set_hostname(&ssl_, host) != 0) {
        return false;
    }

    // Hand mbedtls the stream that is already connected - and, for STARTTLS,
    // has already carried a plaintext exchange. This is the step
    // WiFiClientSecure cannot do, and routing it through ByteStream is what
    // lets the cellular path share it.
    mbedtls_ssl_set_bio(&ssl_, stream_, BioSend, BioRecv, NULL);

    // Bounded. A polled transport reports "nothing yet" as WANT_READ, so an
    // unbounded retry loop would spin here forever against a server that never
    // completes the handshake - and the mail task carries no watchdog, so the
    // device would stop sending mail entirely rather than failing this one
    // send and retrying on the backoff.
    int status = 0;
    const uint32_t handshake_start = millis();
    do {
        status = mbedtls_ssl_handshake(&ssl_);
        if (static_cast<uint32_t>(millis() - handshake_start) >=
            kSmtpHandshakeDeadlineMs) {
            return false;
        }
    } while (status == MBEDTLS_ERR_SSL_WANT_READ ||
             status == MBEDTLS_ERR_SSL_WANT_WRITE);
    if (status != 0) {
        return false;
    }
    if (mbedtls_ssl_get_verify_result(&ssl_) != 0) {
        return false;
    }

    tls_active_ = true;
    // Anything buffered from before the upgrade belongs to the plaintext
    // phase and must not be read as if it arrived over TLS.
    rx_len_ = 0;
    rx_pos_ = 0;
    return true;
}

void SmtpClient::EndTls() {
    if (!tls_ready_) {
        return;
    }
    if (tls_active_) {
        mbedtls_ssl_close_notify(&ssl_);
        tls_active_ = false;
    }
    mbedtls_ssl_free(&ssl_);
    mbedtls_ssl_config_free(&conf_);
    mbedtls_ctr_drbg_free(&drbg_);
    mbedtls_entropy_free(&entropy_);
    tls_ready_ = false;
}

void SmtpClient::Close() {
    EndTls();
    if (stream_ != NULL) {
        stream_->Close();
    }
    rx_len_ = 0;
    rx_pos_ = 0;
}

bool SmtpClient::WriteAll(const char* data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        int n = 0;
        if (tls_active_) {
            n = mbedtls_ssl_write(
                &ssl_, reinterpret_cast<const unsigned char*>(&data[sent]),
                len - sent);
            if (n == MBEDTLS_ERR_SSL_WANT_READ ||
                n == MBEDTLS_ERR_SSL_WANT_WRITE) {
                continue;
            }
        } else {
            return stream_->WriteAll(&data[sent], len - sent);
        }
        if (n <= 0) {
            return false;
        }
        sent += static_cast<size_t>(n);
    }
    return true;
}

bool SmtpClient::ReadLine(char* out, size_t out_len) {
    size_t written = 0;
    const uint32_t started_ms = millis();
    for (;;) {
        if (rx_pos_ >= rx_len_) {
            int n = 0;
            if (tls_active_) {
                n = mbedtls_ssl_read(&ssl_,
                                     reinterpret_cast<unsigned char*>(rx_),
                                     sizeof(rx_));
                if (n == MBEDTLS_ERR_SSL_WANT_READ ||
                    n == MBEDTLS_ERR_SSL_WANT_WRITE) {
                    continue;
                }
            } else {
                n = stream_->Read(reinterpret_cast<uint8_t*>(rx_),
                                  sizeof(rx_));
                if (n == 0) {
                    // Nothing yet, connection still open. A polled transport
                    // reports this routinely; only the overall deadline ends
                    // the wait.
                    if (static_cast<uint32_t>(millis() - started_ms) >=
                        kSmtpReplyDeadlineMs) {
                        return false;
                    }
                    continue;
                }
            }
            if (n <= 0) {
                return false;  // closed or error
            }
            rx_len_ = static_cast<size_t>(n);
            rx_pos_ = 0;
        }

        const char c = rx_[rx_pos_++];
        if (c == '\n') {
            out[written] = '\0';
            return true;
        }
        if (c == '\r') {
            continue;
        }
        if (written + 1 < out_len) {
            out[written++] = c;
        }
        // A line longer than the buffer is truncated rather than failing: the
        // reply code is in the first three characters, which is what decides
        // anything.
    }
}

bool SmtpClient::StreamBody(const MimeMessage& message, size_t* bytes_sent) {
    MimeWriter writer;
    if (!writer.Begin(message)) {
        return false;
    }

    char chunk[kStreamChunkBytes];
    size_t n = 0;
    while ((n = writer.Next(chunk, sizeof(chunk))) > 0) {
        if (!WriteAll(chunk, n)) {
            return false;
        }
        if (bytes_sent != NULL) {
            *bytes_sent += n;
        }
    }

    // End of DATA. The leading CRLF guarantees the dot starts its own line even
    // if the body did not end with a newline.
    return WriteAll("\r\n.\r\n", 5);
}

SmtpResult SmtpClient::Send(const EmailConfig& config,
                            const char* client_hostname,
                            const MimeMessage& message, ByteStream* stream) {
    SmtpResult result;
    if (stream == NULL) {
        result.stage = kSmtpStageConnect;
        return result;
    }
    stream_ = stream;
    tls_active_ = false;
    rx_len_ = 0;
    rx_pos_ = 0;

    SmtpSession session;
    session.Begin(config, client_hostname, message.recipients,
                  message.recipient_count);

    // Opening covers name resolution and the connect, and the transport
    // distinguishes the two - a SocketStream reports which of them it reached,
    // and a ModemStream has already registered before it gets here.
    result.stage = kSmtpStageConnect;
    if (!stream_->Open(config.server, config.port)) {
        // Name which of the two it was. "Cannot resolve the
        // server name" and "the server refused the connection" send a
        // technician to different places.
        result.stage = (stream_->stage() == kStreamDns) ? kSmtpStageDns
                                                        : kSmtpStageConnect;
        session.FailTransport(result.stage);
        return result;
    }

    // Implicit TLS negotiates before a single SMTP byte is exchanged.
    if (config.security == kSmtpSecuritySsl) {
        result.stage = kSmtpStageTlsHandshake;
        if (!BeginTls(config.server)) {
            Close();
            return result;
        }
    }

    char line[512];
    const char* reply = NULL;
    for (int guard = 0; guard < 128; guard++) {
        const SmtpAction action = session.Step(reply);
        reply = NULL;
        result.stage = session.stage();

        switch (action) {
            case kSmtpReadMore:
                if (!ReadLine(line, sizeof(line))) {
                    Close();
                    return result;
                }
                reply = line;
                break;

            case kSmtpSend:
                // QUIT is a courtesy, not part of delivery. By the time the
                // session reaches it the server has already answered 250 to
                // the message body, so the mail is accepted and nothing that
                // happens now can un-accept it. Most servers - Office 365
                // among them - close the connection immediately, so insisting
                // on reading the 221 reports a delivered message as a failure
                // and, worse, has the alert queue retry it and deliver it
                // twice. Write it and finish.
                if (session.delivered()) {
                    WriteAll(session.command(), session.command_len());
                    result.ok = true;
                    result.stage = kSmtpStageDone;
                    result.accepted = session.accepted_recipients();
                    result.rejected = session.rejected_recipients();
                    Close();
                    return result;
                }

                // command_is_secret() is why nothing here logs the command.
                if (!WriteAll(session.command(), session.command_len())) {
                    Close();
                    return result;
                }
                if (!ReadLine(line, sizeof(line))) {
                    Close();
                    return result;
                }
                reply = line;
                break;

            case kSmtpUpgradeTls:
                result.stage = kSmtpStageTlsHandshake;
                if (!BeginTls(config.server)) {
                    Close();
                    return result;
                }
                break;

            case kSmtpSendBody:
                if (!StreamBody(message, &result.bytes_sent)) {
                    result.stage = kSmtpStageBody;
                    Close();
                    return result;
                }
                if (!ReadLine(line, sizeof(line))) {
                    Close();
                    return result;
                }
                reply = line;
                break;

            case kSmtpDone:
                result.ok = true;
                result.stage = kSmtpStageDone;
                result.code = session.last_code();
                result.accepted = session.accepted_recipients();
                result.rejected = session.rejected_recipients();
                Close();
                return result;

            case kSmtpFail:
            default:
                result.stage = session.stage();
                result.code = session.last_code();
                result.accepted = session.accepted_recipients();
                result.rejected = session.rejected_recipients();
                Close();
                return result;
        }
    }

    // The conversation should never take this many steps; treat it as a
    // misbehaving server rather than looping forever.
    Close();
    return result;
}

}  // namespace gwmail
