#include "mail_smtp_session.h"

#include <stdio.h>

#include "mail_base64.h"

namespace gwmail {
namespace {

bool IsDigit(char c) { return c >= '0' && c <= '9'; }

size_t CopyBounded(char* out, size_t out_len, const char* text) {
    size_t i = 0;
    if (out_len == 0) {
        return 0;
    }
    while (text != NULL && text[i] != '\0' && i + 1 < out_len) {
        out[i] = text[i];
        i++;
    }
    out[i] = '\0';
    return i;
}

}  // namespace

const char* SmtpStageText(SmtpStage stage) {
    switch (stage) {
        case kSmtpStageIdle: return "idle";
        case kSmtpStageDns: return "DNS lookup";
        case kSmtpStageConnect: return "TCP connect";
        case kSmtpStageTlsHandshake: return "TLS handshake";
        case kSmtpStageGreeting: return "server greeting";
        case kSmtpStageEhlo: return "EHLO";
        case kSmtpStageStartTls: return "STARTTLS";
        case kSmtpStageAuth: return "authentication";
        case kSmtpStageMailFrom: return "sender address";
        case kSmtpStageRcptTo: return "recipient address";
        case kSmtpStageData: return "DATA";
        case kSmtpStageBody: return "message body";
        case kSmtpStageQuit: return "QUIT";
        case kSmtpStageDone: return "done";
        default: return "unknown";
    }
}

bool ParseSmtpReply(const char* line, int* code, bool* is_final) {
    if (line == NULL || code == NULL || is_final == NULL) {
        return false;
    }
    if (!IsDigit(line[0]) || !IsDigit(line[1]) || !IsDigit(line[2])) {
        return false;
    }
    *code = ((line[0] - '0') * 100) + ((line[1] - '0') * 10) + (line[2] - '0');
    // RFC 5321: a hyphen in the fourth position means more lines follow. A line
    // that ends right after the code is a complete reply.
    *is_final = (line[3] != '-');
    return true;
}

SmtpSession::SmtpSession()
    : config_(NULL),
      recipients_(NULL),
      recipient_count_(0),
      command_len_(0),
      command_is_secret_(false),
      stage_(kSmtpStageIdle),
      last_code_(0),
      auth_step_(0),
      rcpt_index_(0),
      accepted_(0),
      rejected_(0),
      delivered_(false),
      tls_active_(false),
      ehlo_sent_(false) {
    client_hostname_[0] = '\0';
    command_[0] = '\0';
}

void SmtpSession::Begin(const EmailConfig& config, const char* client_hostname,
                        const EmailRecipient* const* recipients,
                        int recipient_count) {
    config_ = &config;
    recipients_ = recipients;
    recipient_count_ = recipient_count;
    CopyBounded(client_hostname_, sizeof(client_hostname_),
                (client_hostname != NULL && client_hostname[0] != '\0')
                    ? client_hostname
                    : "gateway");

    command_[0] = '\0';
    command_len_ = 0;
    command_is_secret_ = false;
    stage_ = kSmtpStageGreeting;
    last_code_ = 0;
    auth_step_ = 0;
    rcpt_index_ = 0;
    accepted_ = 0;
    rejected_ = 0;
    delivered_ = false;
    // Implicit TLS is already established by the time the greeting arrives, so
    // STARTTLS must not be attempted on top of it.
    tls_active_ = (config.security == kSmtpSecuritySsl);
    ehlo_sent_ = false;
}

SmtpAction SmtpSession::Fail(SmtpStage stage, int code) {
    stage_ = stage;
    last_code_ = code;
    command_len_ = 0;
    command_is_secret_ = false;
    return kSmtpFail;
}

void SmtpSession::FailTransport(SmtpStage stage) {
    stage_ = stage;
    last_code_ = 0;
}

SmtpAction SmtpSession::Emit(const char* text, bool secret) {
    command_len_ = CopyBounded(command_, sizeof(command_), text);
    command_is_secret_ = secret;
    return kSmtpSend;
}

SmtpAction SmtpSession::EmitPair(const char* prefix, const char* value,
                                 const char* suffix) {
    const int n = snprintf(command_, sizeof(command_), "%s%s%s", prefix, value,
                           suffix);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(command_)) {
        // Cannot happen with validated field lengths, but a truncated command
        // would be sent as a different command, so refuse rather than send it.
        return Fail(stage_, 0);
    }
    command_len_ = static_cast<size_t>(n);
    command_is_secret_ = false;
    return kSmtpSend;
}

SmtpAction SmtpSession::AfterEhlo() {
    // STARTTLS before AUTH: credentials must never cross a plaintext socket.
    if (config_->security == kSmtpSecurityStartTls && !tls_active_) {
        stage_ = kSmtpStageStartTls;
        return Emit("STARTTLS\r\n", false);
    }
    if (EmailUsesAuth(*config_)) {
        stage_ = kSmtpStageAuth;
        auth_step_ = 0;
        return Emit("AUTH LOGIN\r\n", false);
    }
    stage_ = kSmtpStageMailFrom;
    return EmitPair("MAIL FROM:<", config_->from, ">\r\n");
}

SmtpAction SmtpSession::SendNextRecipient() {
    while (rcpt_index_ < recipient_count_) {
        const EmailRecipient* r = recipients_[rcpt_index_];
        if (r != NULL && r->configured()) {
            stage_ = kSmtpStageRcptTo;
            return EmitPair("RCPT TO:<", r->address, ">\r\n");
        }
        rcpt_index_++;
    }

    if (accepted_ == 0) {
        // Every address was refused. There is nobody to send to, and the code
        // from the last refusal is the useful thing to report.
        return Fail(kSmtpStageRcptTo, last_code_);
    }
    stage_ = kSmtpStageData;
    return Emit("DATA\r\n", false);
}

SmtpAction SmtpSession::Step(const char* reply) {
    if (config_ == NULL) {
        return Fail(kSmtpStageIdle, 0);
    }

    // No reply pending. Either the transport has just connected and must read
    // the greeting, or it has just upgraded to TLS and owes us an EHLO.
    if (reply == NULL) {
        if (stage_ == kSmtpStageEhlo && !ehlo_sent_) {
            ehlo_sent_ = true;
            return EmitPair("EHLO ", client_hostname_, "\r\n");
        }
        return kSmtpReadMore;
    }

    int code = 0;
    bool is_final = false;
    if (!ParseSmtpReply(reply, &code, &is_final)) {
        return Fail(stage_, 0);
    }
    if (!is_final) {
        // A continuation line. EHLO's capability list arrives this way and none
        // of it changes what we do next: STARTTLS and AUTH are driven by the
        // configuration, so that the failure is "the server refused STARTTLS"
        // rather than a silent downgrade to plaintext.
        return kSmtpReadMore;
    }
    last_code_ = code;

    switch (stage_) {
        case kSmtpStageGreeting:
            if (code != 220) {
                return Fail(kSmtpStageGreeting, code);
            }
            stage_ = kSmtpStageEhlo;
            ehlo_sent_ = true;
            return EmitPair("EHLO ", client_hostname_, "\r\n");

        case kSmtpStageEhlo:
            if (code != 250) {
                return Fail(kSmtpStageEhlo, code);
            }
            return AfterEhlo();

        case kSmtpStageStartTls:
            if (code != 220) {
                return Fail(kSmtpStageStartTls, code);
            }
            tls_active_ = true;
            // RFC 3207: the session resets after the upgrade, so EHLO must be
            // issued again on the secure channel.
            stage_ = kSmtpStageEhlo;
            ehlo_sent_ = false;
            return kSmtpUpgradeTls;

        case kSmtpStageAuth:
            if (auth_step_ == 0) {
                if (code != 334) {
                    return Fail(kSmtpStageAuth, code);
                }
                auth_step_ = 1;
                char encoded[kSmtpCommandChars];
                if (Base64EncodeString(config_->username, encoded,
                                       sizeof(encoded) - 2) == 0) {
                    return Fail(kSmtpStageAuth, 0);
                }
                const int n = snprintf(command_, sizeof(command_), "%s\r\n",
                                       encoded);
                if (n < 0 || static_cast<size_t>(n) >= sizeof(command_)) {
                    return Fail(kSmtpStageAuth, 0);
                }
                command_len_ = static_cast<size_t>(n);
                // Marked secret as well as the password. It is a stored field
                // value inside a credential exchange, and base64 is not
                // encryption - a log line carrying it is a log line carrying
                // the account name.
                command_is_secret_ = true;
                return kSmtpSend;
            }
            if (auth_step_ == 1) {
                if (code != 334) {
                    return Fail(kSmtpStageAuth, code);
                }
                auth_step_ = 2;
                char encoded[kSmtpCommandChars];
                if (Base64EncodeString(config_->password, encoded,
                                       sizeof(encoded) - 2) == 0) {
                    return Fail(kSmtpStageAuth, 0);
                }
                const int n = snprintf(command_, sizeof(command_), "%s\r\n",
                                       encoded);
                if (n < 0 || static_cast<size_t>(n) >= sizeof(command_)) {
                    return Fail(kSmtpStageAuth, 0);
                }
                command_len_ = static_cast<size_t>(n);
                // The only command in the conversation that must never reach a
                // log, however verbose the diagnostics get.
                command_is_secret_ = true;
                return kSmtpSend;
            }
            // 235 is success; 535 here is usually a provider that wants an
            // app-specific password rather than the account password, and reporting the code is what makes
            // that diagnosable on site.
            if (code != 235) {
                return Fail(kSmtpStageAuth, code);
            }
            command_is_secret_ = false;
            stage_ = kSmtpStageMailFrom;
            return EmitPair("MAIL FROM:<", config_->from, ">\r\n");

        case kSmtpStageMailFrom:
            if (code != 250) {
                // 550 here is almost always the sender not matching the
                // authenticated account.
                return Fail(kSmtpStageMailFrom, code);
            }
            rcpt_index_ = 0;
            return SendNextRecipient();

        case kSmtpStageRcptTo:
            if (code == 250 || code == 251) {
                accepted_++;
            } else {
                rejected_++;
            }
            rcpt_index_++;
            return SendNextRecipient();

        case kSmtpStageData:
            if (code != 354) {
                return Fail(kSmtpStageData, code);
            }
            stage_ = kSmtpStageBody;
            return kSmtpSendBody;

        case kSmtpStageBody:
            if (code != 250) {
                return Fail(kSmtpStageBody, code);
            }
            // The message is now the server's responsibility.
            delivered_ = true;
            stage_ = kSmtpStageQuit;
            return Emit("QUIT\r\n", false);

        case kSmtpStageQuit:
            // The message was accepted at the body stage; whatever the server
            // says to QUIT cannot un-accept it, so this never fails.
            stage_ = kSmtpStageDone;
            return kSmtpDone;

        default:
            return Fail(stage_, code);
    }
}

}  // namespace gwmail
