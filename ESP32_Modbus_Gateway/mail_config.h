// Email alert configuration, set from the Settings page.
//
// Two things in this file are security-relevant rather than cosmetic:
//
//  - The password is write-only from the Settings page's perspective. It is
//    stored, never rendered back, and never logged - log a field name, never
//    a field value.
//  - Addresses and the subject prefix are validated for CR and LF. Both end up
//    in message headers, and a newline in either would let whatever follows be
//    injected as a header of its own.
//
// Plain fixed-size char arrays rather than Arduino Strings: the alert task
// takes a private copy of the whole struct under a mutex before each send, and
// a String member would make that copy allocate.

#ifndef GWMAIL_CONFIG_H
#define GWMAIL_CONFIG_H

#include <stddef.h>
#include <stdint.h>

namespace gwmail {

static const size_t kEmailHostChars = 65;  // 64 + NUL
static const size_t kEmailAddressChars = 65;
static const size_t kEmailUserChars = 65;
static const size_t kEmailPasswordChars = 65;
static const size_t kSubjectPrefixChars = 33;

// One message is addressed to every recipient rather than one message each,
// so an event costs one slot of the alert queue whatever the recipient count.
static const int kMaxRecipients = 4;

// None is offered because some site relays genuinely do not support TLS, but
// the Settings page warns when it is selected.
enum SmtpSecurity {
    kSmtpSecurityNone = 0,
    kSmtpSecurityStartTls = 1,
    kSmtpSecuritySsl = 2,
};

struct EmailRecipient {
    char address[kEmailAddressChars];

    EmailRecipient() { address[0] = '\0'; }

    bool configured() const { return address[0] != '\0'; }
};

struct EmailConfig {
    bool enabled;
    char server[kEmailHostChars];
    uint16_t port;
    uint8_t security;  // SmtpSecurity
    char username[kEmailUserChars];
    char password[kEmailPasswordChars];
    char from[kEmailAddressChars];
    EmailRecipient recipients[kMaxRecipients];
    char subject_prefix[kSubjectPrefixChars];
    // Allow sending over the cellular modem when WiFi is unavailable.
    bool send_on_cellular;

    EmailConfig();
};

enum EmailValidation {
    kEmailOk = 0,
    kEmailNoServer,
    kEmailBadPort,
    kEmailBadSecurity,
    kEmailNoFrom,
    kEmailBadFrom,
    kEmailNoRecipient,
    kEmailBadRecipient,
    kEmailBadSubjectPrefix,
    kEmailPasswordWithoutUser,
};

const char* EmailValidationText(EmailValidation result);

// Validated only when enabled: every field is kept while alerts are switched
// off, so a half-filled configuration must not block saving it.
EmailValidation ValidateEmailConfig(const EmailConfig& config);

// Minimal address sanity: exactly one '@' with something either side, no
// spaces, no controls, no CR or LF. This is not RFC 5321 validation - it is the
// subset that keeps a header injection out of the message and catches the
// typos a technician actually makes.
bool IsPlausibleEmailAddress(const char* address);

// True if `text` is safe to place in a header: no CR, no LF, no other control
// characters.
bool IsHeaderSafe(const char* text);

// Collect the configured recipients into `out`. Returns how many were written.
int SelectRecipients(const EmailConfig& config, const EmailRecipient** out,
                     int max_out);

// True when authentication should be attempted at all. An empty username means
// an internal relay that wants no AUTH, which is what many industrial sites
// actually use.
bool EmailUsesAuth(const EmailConfig& config);

}  // namespace gwmail

#endif  // GWMAIL_CONFIG_H
