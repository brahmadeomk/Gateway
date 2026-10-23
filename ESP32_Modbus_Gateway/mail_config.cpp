#include "mail_config.h"

namespace gwmail {
namespace {

void ClearChars(char* out, size_t len) {
    for (size_t i = 0; i < len; i++) {
        out[i] = '\0';
    }
}

}  // namespace

EmailConfig::EmailConfig()
    : enabled(false),
      port(587),  // STARTTLS, what most providers expect
      security(kSmtpSecurityStartTls) {
    ClearChars(server, kEmailHostChars);
    ClearChars(username, kEmailUserChars);
    ClearChars(password, kEmailPasswordChars);
    ClearChars(from, kEmailAddressChars);
    ClearChars(subject_prefix, kSubjectPrefixChars);
}

const char* EmailValidationText(EmailValidation result) {
    switch (result) {
        case kEmailOk: return "ok";
        case kEmailNoServer: return "SMTP server is missing or invalid";
        case kEmailBadPort: return "SMTP port is invalid";
        case kEmailBadSecurity: return "security mode is invalid";
        case kEmailNoFrom: return "From address is missing";
        case kEmailBadFrom: return "From address is not a valid email address";
        case kEmailNoRecipient: return "at least one recipient is required";
        case kEmailBadRecipient: return "a recipient is not a valid email address";
        case kEmailBadSubjectPrefix: return "subject prefix contains control characters";
        case kEmailPasswordWithoutUser: return "a password needs a username";
        default: return "unknown error";
    }
}

bool IsHeaderSafe(const char* text) {
    if (text == NULL) {
        return false;
    }
    for (size_t i = 0; text[i] != '\0'; i++) {
        const unsigned char c = static_cast<unsigned char>(text[i]);
        // Rejecting every control character covers CR and LF, which are the
        // two that would let the rest of the field become a header of its own.
        if (c < 32 || c == 127) {
            return false;
        }
    }
    return true;
}

bool IsPlausibleEmailAddress(const char* address) {
    if (address == NULL || address[0] == '\0' || !IsHeaderSafe(address)) {
        return false;
    }

    int at_count = 0;
    size_t at_index = 0;
    size_t len = 0;
    for (; address[len] != '\0'; len++) {
        const char c = address[len];
        if (c == ' ' || c == '<' || c == '>' || c == ',' || c == ';') {
            return false;
        }
        if (c == '@') {
            at_count++;
            at_index = len;
        }
    }

    if (at_count != 1) {
        return false;
    }
    if (at_index == 0 || at_index + 1 >= len) {
        return false;  // nothing before or after the '@'
    }
    // A domain with no dot is legal on an internal relay ("relay@plant"), so
    // that is deliberately not required here.
    return true;
}

int SelectRecipients(const EmailConfig& config, const EmailRecipient** out,
                     int max_out) {
    if (out == NULL || max_out <= 0) {
        return 0;
    }
    int count = 0;
    for (int i = 0; i < kMaxRecipients && count < max_out; i++) {
        if (config.recipients[i].configured()) {
            out[count++] = &config.recipients[i];
        }
    }
    return count;
}

bool EmailUsesAuth(const EmailConfig& config) {
    return config.username[0] != '\0';
}

EmailValidation ValidateEmailConfig(const EmailConfig& config) {
    if (!config.enabled) {
        return kEmailOk;
    }

    if (config.server[0] == '\0' || !IsHeaderSafe(config.server)) {
        return kEmailNoServer;
    }
    if (config.port == 0) {
        return kEmailBadPort;
    }
    if (config.security > kSmtpSecuritySsl) {
        return kEmailBadSecurity;
    }
    if (config.from[0] == '\0') {
        return kEmailNoFrom;
    }
    if (!IsPlausibleEmailAddress(config.from)) {
        return kEmailBadFrom;
    }
    if (!IsHeaderSafe(config.subject_prefix)) {
        return kEmailBadSubjectPrefix;
    }
    // A password with no username cannot be used: AUTH is skipped entirely
    // when the username is empty, so the credential would sit in flash doing
    // nothing while the technician believed authentication was configured.
    if (config.password[0] != '\0' && config.username[0] == '\0') {
        return kEmailPasswordWithoutUser;
    }

    int configured = 0;
    for (int i = 0; i < kMaxRecipients; i++) {
        if (!config.recipients[i].configured()) {
            continue;
        }
        if (!IsPlausibleEmailAddress(config.recipients[i].address)) {
            return kEmailBadRecipient;
        }
        configured++;
    }
    if (configured == 0) {
        return kEmailNoRecipient;
    }
    return kEmailOk;
}

}  // namespace gwmail
