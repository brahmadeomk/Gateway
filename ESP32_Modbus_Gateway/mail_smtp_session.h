// SMTP conversation as a state machine, with no socket in sight.
//
// The transport belongs in platform/ (DNS, TCP, TLS, timeouts). What is here is
// the part that decides: which command comes next, whether a reply was
// acceptable, and - the point of the whole exercise - exactly which stage
// failed when one does.
//
// That last part matters most: a test send must report DNS resolution, TCP
// connect, TLS handshake, authentication or server rejection with its
// response code, not a generic failure - it is what lets a technician fix a
// mail problem on site instead of escalating it. A state machine in core/ is how that reporting gets tested
// against real reply sequences without a mail server.
//
// Usage from the transport:
//
//   session.Begin(...);
//   for (;;) {
//       switch (session.Step(reply_or_NULL)) {
//           case kSmtpReadMore:   read one reply line, pass it next time
//           case kSmtpSend:       write command(), then read a reply line
//           case kSmtpUpgradeTls: upgrade the socket, then Step(NULL)
//           case kSmtpSendBody:   stream the message and CRLF "." CRLF,
//                                 then read a reply line
//           case kSmtpDone:       accepted
//           case kSmtpFail:       stage() and last_code() say why
//       }
//   }

#ifndef GWMAIL_SMTP_SESSION_H
#define GWMAIL_SMTP_SESSION_H

#include <stddef.h>

#include "mail_config.h"

namespace gwmail {

// The stages a send passes through, in order. A failure reports the stage it
// reached, which is what the portal shows and what the Status Log records.
//
// The first three are never entered by this class - the transport owns them -
// but they are named here so there is one vocabulary for "where did it fail"
// across the Settings page and the Status Log.
enum SmtpStage {
    kSmtpStageIdle = 0,
    kSmtpStageDns,
    kSmtpStageConnect,
    kSmtpStageTlsHandshake,
    kSmtpStageGreeting,
    kSmtpStageEhlo,
    kSmtpStageStartTls,
    kSmtpStageAuth,
    kSmtpStageMailFrom,
    kSmtpStageRcptTo,
    kSmtpStageData,
    kSmtpStageBody,
    kSmtpStageQuit,
    kSmtpStageDone,
};

// Short, human-readable stage name for the portal and the log.
const char* SmtpStageText(SmtpStage stage);

// Parse a reply line. `code` receives the three-digit code, `is_final` is false
// for a continuation line ("250-STARTTLS") and true for the last one
// ("250 HELP"). Returns false if the line is not a reply at all.
bool ParseSmtpReply(const char* line, int* code, bool* is_final);

enum SmtpAction {
    kSmtpReadMore = 0,
    kSmtpSend,
    kSmtpUpgradeTls,
    kSmtpSendBody,
    kSmtpDone,
    kSmtpFail,
};

// Longest command this builds: "RCPT TO:<...>\r\n" with a 64-character
// address, or a base64 password of the same length. 192 leaves room to spare.
static const size_t kSmtpCommandChars = 192;

class SmtpSession {
  public:
    SmtpSession();

    // `recipients` are the addresses this message is going to. `client_hostname` identifies this device in EHLO.
    void Begin(const EmailConfig& config, const char* client_hostname,
               const EmailRecipient* const* recipients, int recipient_count);

    // Advance the conversation. `reply` is the server's most recent line, or
    // NULL when none is pending: the first call, and after a TLS upgrade or a
    // body stream.
    SmtpAction Step(const char* reply);

    const char* command() const { return command_; }
    size_t command_len() const { return command_len_; }

    // True when command() holds credential material. Every diagnostic path
    // must consult this before printing a command - log a field name, never
    // a field value.
    bool command_is_secret() const { return command_is_secret_; }

    SmtpStage stage() const { return stage_; }

    // True once the server has answered 250 to the message body.
    //
    // From that moment the mail is delivered and nothing later can un-deliver
    // it. QUIT is a courtesy, and most servers close the connection the
    // instant they see it - so a transport that insists on reading the 221
    // reply reports a delivered message as a failure, and the alert queue
    // retries it and delivers it twice.
    bool delivered() const { return delivered_; }

    // The server's code at the point of failure, or 0 if the failure was not a
    // server rejection. Reported verbatim on the Settings page.
    int last_code() const { return last_code_; }

    // Recipients the server accepted and rejected. A message goes ahead if at
    // least one was accepted; one bad address out of four should not silence
    // the alert for the other three.
    int accepted_recipients() const { return accepted_; }
    int rejected_recipients() const { return rejected_; }

    // Set by the transport before Step() is first called, so a failure at DNS,
    // connect or handshake reports that stage rather than "greeting".
    void SetTransportStage(SmtpStage stage) { stage_ = stage; }
    void FailTransport(SmtpStage stage);

  private:
    SmtpAction Fail(SmtpStage stage, int code);
    SmtpAction Emit(const char* text, bool secret);
    SmtpAction EmitPair(const char* prefix, const char* value,
                        const char* suffix);
    SmtpAction AfterEhlo();
    SmtpAction SendNextRecipient();

    const EmailConfig* config_;
    const EmailRecipient* const* recipients_;
    int recipient_count_;
    char client_hostname_[kEmailHostChars];

    char command_[kSmtpCommandChars];
    size_t command_len_;
    bool command_is_secret_;

    SmtpStage stage_;
    int last_code_;
    int auth_step_;
    int rcpt_index_;
    int accepted_;
    int rejected_;
    bool delivered_;
    bool tls_active_;
    bool ehlo_sent_;
};

}  // namespace gwmail

#endif  // GWMAIL_SMTP_SESSION_H
