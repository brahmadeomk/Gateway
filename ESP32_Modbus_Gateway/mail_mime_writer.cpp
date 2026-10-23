#include "mail_mime_writer.h"

#include <stdio.h>

namespace gwmail {
namespace {

// Fixed boundary. It cannot collide with the content: the text body is composed
// from temperatures and field names, and the attachment is base64, whose
// alphabet excludes '='-prefixed runs of this shape at a line start.
const char kBoundary[] = "==GATEWAY-BOUNDARY-7f3a==";

// RFC 2045 caps an encoded line at 76 characters.
const int kBase64LineChars = 76;

size_t TextLength(const char* text) {
    size_t len = 0;
    while (text != NULL && text[len] != '\0') {
        len++;
    }
    return len;
}

}  // namespace

MimeWriter::MimeWriter()
    : headers_len_(0),
      part_len_(0),
      epilogue_len_(0),
      body_(NULL),
      attachment_(NULL),
      attachment_len_(0),
      state_(kStateDone),
      headers_offset_(0),
      body_offset_(0),
      part_offset_(0),
      epilogue_offset_(0),
      source_offset_(0),
      bytes_written_(0),
      base64_column_(0),
      at_line_start_(true),
      base64_finished_(false) {
    headers_[0] = '\0';
    part_[0] = '\0';
    epilogue_[0] = '\0';
}

bool MimeWriter::Begin(const MimeMessage& message) {
    state_ = kStateDone;
    headers_len_ = 0;
    part_len_ = 0;
    epilogue_len_ = 0;
    headers_offset_ = 0;
    body_offset_ = 0;
    part_offset_ = 0;
    epilogue_offset_ = 0;
    source_offset_ = 0;
    bytes_written_ = 0;
    base64_column_ = 0;
    at_line_start_ = true;
    base64_finished_ = false;
    encoder_.Reset();

    if (message.from == NULL || message.from[0] == '\0' ||
        message.recipients == NULL || message.recipient_count <= 0 ||
        message.body == NULL) {
        return false;
    }

    // Every header value that came from configuration is re-checked here. The
    // settings page validates them too, but this is the last point before they
    // become headers, and a CRLF slipping through would split the message.
    if (!IsHeaderSafe(message.from) ||
        (message.subject != NULL && !IsHeaderSafe(message.subject)) ||
        (message.date != NULL && !IsHeaderSafe(message.date))) {
        return false;
    }

    const bool has_attachment =
        (message.attachment != NULL && message.attachment_len > 0);

    int n = snprintf(headers_, sizeof(headers_), "From: <%s>\r\nTo: ",
                     message.from);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(headers_)) {
        return false;
    }
    size_t used = static_cast<size_t>(n);

    int addressed = 0;
    for (int i = 0; i < message.recipient_count; i++) {
        const EmailRecipient* r = message.recipients[i];
        if (r == NULL || !r->configured()) {
            continue;
        }
        if (!IsHeaderSafe(r->address)) {
            return false;
        }
        const int written =
            snprintf(&headers_[used], sizeof(headers_) - used, "%s<%s>",
                     (addressed == 0) ? "" : ", ", r->address);
        if (written < 0 || used + static_cast<size_t>(written) >=
                               sizeof(headers_)) {
            return false;
        }
        used += static_cast<size_t>(written);
        addressed++;
    }
    if (addressed == 0) {
        return false;
    }

    const int tail = snprintf(
        &headers_[used], sizeof(headers_) - used,
        "\r\nSubject: %s\r\n%s%s%sMIME-Version: 1.0\r\n",
        (message.subject != NULL) ? message.subject : "Gateway alert",
        (message.date != NULL) ? "Date: " : "",
        (message.date != NULL) ? message.date : "",
        (message.date != NULL) ? "\r\n" : "");
    if (tail < 0 || used + static_cast<size_t>(tail) >= sizeof(headers_)) {
        return false;
    }
    used += static_cast<size_t>(tail);

    if (has_attachment) {
        const int ct = snprintf(
            &headers_[used], sizeof(headers_) - used,
            "Content-Type: multipart/mixed; boundary=\"%s\"\r\n"
            "\r\n"
            "--%s\r\n"
            "Content-Type: text/plain; charset=us-ascii\r\n"
            "Content-Transfer-Encoding: 7bit\r\n"
            "\r\n",
            kBoundary, kBoundary);
        if (ct < 0 || used + static_cast<size_t>(ct) >= sizeof(headers_)) {
            return false;
        }
        used += static_cast<size_t>(ct);

        const int part = snprintf(
            part_, sizeof(part_),
            "\r\n--%s\r\n"
            "Content-Type: image/bmp; name=\"%s\"\r\n"
            "Content-Transfer-Encoding: base64\r\n"
            "Content-Disposition: attachment; filename=\"%s\"\r\n"
            "\r\n",
            kBoundary,
            (message.attachment_name != NULL) ? message.attachment_name
                                              : "thermal.bmp",
            (message.attachment_name != NULL) ? message.attachment_name
                                              : "thermal.bmp");
        if (part < 0 || static_cast<size_t>(part) >= sizeof(part_)) {
            return false;
        }
        part_len_ = static_cast<size_t>(part);

        const int epi = snprintf(epilogue_, sizeof(epilogue_),
                                 "\r\n--%s--\r\n", kBoundary);
        if (epi < 0 || static_cast<size_t>(epi) >= sizeof(epilogue_)) {
            return false;
        }
        epilogue_len_ = static_cast<size_t>(epi);
    } else {
        const int ct = snprintf(&headers_[used], sizeof(headers_) - used,
                                "Content-Type: text/plain; charset=us-ascii\r\n"
                                "Content-Transfer-Encoding: 7bit\r\n"
                                "\r\n");
        if (ct < 0 || used + static_cast<size_t>(ct) >= sizeof(headers_)) {
            return false;
        }
        used += static_cast<size_t>(ct);
    }

    headers_len_ = used;
    body_ = message.body;
    attachment_ = has_attachment ? message.attachment : NULL;
    attachment_len_ = has_attachment ? message.attachment_len : 0;
    state_ = kStateHeaders;
    return true;
}

size_t MimeWriter::EmitFrom(const char* source, size_t source_len, char* out,
                            size_t out_len, size_t* offset) {
    size_t written = 0;
    while (*offset < source_len && written < out_len) {
        out[written++] = source[(*offset)++];
    }
    return written;
}

size_t MimeWriter::EmitBody(char* out, size_t out_len) {
    const size_t body_len = TextLength(body_);
    size_t written = 0;

    // Two characters of headroom: a stuffed dot and a synthesised CR both need
    // to emit two bytes for one input byte.
    while (body_offset_ < body_len && written + 2 <= out_len) {
        const char c = body_[body_offset_];

        if (at_line_start_ && c == '.') {
            // RFC 5321 transparency: double it, or the server ends DATA here.
            out[written++] = '.';
            out[written++] = '.';
            body_offset_++;
            at_line_start_ = false;
            continue;
        }

        if (c == '\n') {
            // Accept a body written with bare LF and normalise it, so a caller
            // composing text does not have to remember the wire format.
            out[written++] = '\r';
            out[written++] = '\n';
            body_offset_++;
            at_line_start_ = true;
            continue;
        }
        if (c == '\r') {
            // Already CRLF: pass it through as a pair and do not double the LF.
            out[written++] = '\r';
            body_offset_++;
            if (body_offset_ < body_len && body_[body_offset_] == '\n') {
                out[written++] = '\n';
                body_offset_++;
            }
            at_line_start_ = true;
            continue;
        }

        out[written++] = c;
        body_offset_++;
        at_line_start_ = false;
    }
    return written;
}

size_t MimeWriter::EmitBase64(char* out, size_t out_len) {
    size_t written = 0;

    // Each iteration can emit 4 encoded characters plus a CRLF.
    while (written + 6 <= out_len) {
        if (source_offset_ >= attachment_len_) {
            if (base64_finished_) {
                if (base64_column_ > 0) {
                    out[written++] = '\r';
                    out[written++] = '\n';
                    base64_column_ = 0;
                }
                break;
            }
            const size_t n = encoder_.Final(&out[written], out_len - written);
            written += n;
            base64_column_ += static_cast<int>(n);
            base64_finished_ = true;
            continue;
        }

        size_t take = attachment_len_ - source_offset_;
        if (take > 3) {
            take = 3;
        }
        // A partial final group produces nothing here - the encoder carries it
        // and Final() emits it - so the column must follow what was actually
        // written, not what was consumed.
        const size_t produced = encoder_.Update(&attachment_[source_offset_],
                                                take, &out[written],
                                                out_len - written);
        written += produced;
        source_offset_ += take;
        base64_column_ += static_cast<int>(produced);

        if (produced > 0 && base64_column_ >= kBase64LineChars) {
            out[written++] = '\r';
            out[written++] = '\n';
            base64_column_ = 0;
        }
    }
    return written;
}

size_t MimeWriter::Next(char* out, size_t out_len) {
    if (out == NULL || out_len < 8 || state_ == kStateDone) {
        return 0;
    }

    size_t written = 0;
    while (written < out_len && state_ != kStateDone) {
        size_t produced = 0;
        switch (state_) {
            case kStateHeaders:
                produced = EmitFrom(headers_, headers_len_, &out[written],
                                    out_len - written, &headers_offset_);
                if (headers_offset_ >= headers_len_) {
                    state_ = kStateBody;
                }
                break;

            case kStateBody:
                produced = EmitBody(&out[written], out_len - written);
                if (body_offset_ >= TextLength(body_)) {
                    state_ = (attachment_ != NULL) ? kStatePartHeaders
                                                   : kStateDone;
                }
                break;

            case kStatePartHeaders:
                produced = EmitFrom(part_, part_len_, &out[written],
                                    out_len - written, &part_offset_);
                if (part_offset_ >= part_len_) {
                    state_ = kStateBase64;
                }
                break;

            case kStateBase64:
                produced = EmitBase64(&out[written], out_len - written);
                if (base64_finished_ && base64_column_ == 0) {
                    state_ = kStateEpilogue;
                }
                break;

            case kStateEpilogue:
                produced = EmitFrom(epilogue_, epilogue_len_, &out[written],
                                    out_len - written, &epilogue_offset_);
                if (epilogue_offset_ >= epilogue_len_) {
                    state_ = kStateDone;
                }
                break;

            default:
                state_ = kStateDone;
                break;
        }

        written += produced;
        if (produced == 0) {
            // No room left for this state's smallest unit of progress. Return
            // what there is; the caller comes back with an empty buffer.
            break;
        }
    }

    bytes_written_ += written;
    return written;
}

}  // namespace gwmail
