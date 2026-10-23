// MIME assembly for the alert email, as a pull-style stream.
//
// Base64 is encoded into the MIME body in chunks rather than assembled as
// one string in a single allocation. So this
// never holds the message. The caller offers a buffer, gets the next slice, and
// writes it to the socket:
//
//   writer.Begin(message);
//   while ((n = writer.Next(buffer, sizeof(buffer))) > 0) client.write(buffer, n);
//
// A 12 KB attachment becomes 16 KB of base64, and building that on top of the body
// as one String is exactly the heap spike this firmware exists without.
//
// Two details that are easy to miss and produce a message a mail server
// silently mangles:
//
//  - Every line ends CRLF, including inside the body.
//  - A body line beginning with '.' is doubled. RFC 5321 ends the DATA phase at
//    a lone dot on its own line, so an undoubled one truncates the message
//    there. Base64 output never starts a line with '.', but the text body is
//    composed from field values and must be stuffed.

#ifndef GWMAIL_MIME_WRITER_H
#define GWMAIL_MIME_WRITER_H

#include <stddef.h>
#include <stdint.h>

#include "mail_base64.h"
#include "mail_config.h"

namespace gwmail {

// Header block plus the first part's headers. Bounded by 4 recipients at 64
// characters, a 64-character sender and a subject line.
static const size_t kMimeHeaderChars = 768;
static const size_t kMimePartHeaderChars = 256;

struct MimeMessage {
    const char* from;
    const EmailRecipient* const* recipients;
    int recipient_count;
    const char* subject;
    const char* body;  // plain text, CRLF or LF line endings both accepted

    // Optional attachment, or NULL for a text-only message (all the
    // gateway's alerts are text-only today).
    const uint8_t* attachment;
    size_t attachment_len;
    const char* attachment_name;

    // RFC 5322 date, e.g. "Mon, 21 Aug 2026 14:03:11 +0000". NULL omits the
    // header, which servers generally fill in themselves - the device has no
    // wall clock until SNTP has synced.
    const char* date;

    MimeMessage()
        : from(NULL),
          recipients(NULL),
          recipient_count(0),
          subject(NULL),
          body(NULL),
          attachment(NULL),
          attachment_len(0),
          attachment_name("thermal.bmp"),
          date(NULL) {}
};

class MimeWriter {
  public:
    MimeWriter();

    // Compose the headers and prepare to stream. Returns false if the headers
    // do not fit or a required field is missing, in which case Next() yields
    // nothing - better than sending a message with a malformed header block.
    bool Begin(const MimeMessage& message);

    // Write the next slice into `out`. Returns the number of bytes written, or
    // 0 when the message is complete. Needs at least 8 bytes of room to make
    // progress.
    size_t Next(char* out, size_t out_len);

    bool done() const { return state_ == kStateDone; }

    // Total bytes produced so far, for the Status Log.
    size_t bytes_written() const { return bytes_written_; }

  private:
    enum State {
        kStateHeaders = 0,
        kStateBody,
        kStatePartHeaders,
        kStateBase64,
        kStateEpilogue,
        kStateDone,
    };

    size_t EmitFrom(const char* source, size_t source_len, char* out,
                    size_t out_len, size_t* offset);
    size_t EmitBody(char* out, size_t out_len);
    size_t EmitBase64(char* out, size_t out_len);

    char headers_[kMimeHeaderChars];
    size_t headers_len_;
    char part_[kMimePartHeaderChars];
    size_t part_len_;
    char epilogue_[kMimePartHeaderChars];
    size_t epilogue_len_;

    const char* body_;
    const uint8_t* attachment_;
    size_t attachment_len_;

    State state_;
    size_t headers_offset_;
    size_t body_offset_;
    size_t part_offset_;
    size_t epilogue_offset_;
    size_t source_offset_;
    size_t bytes_written_;

    Base64Encoder encoder_;
    int base64_column_;
    bool at_line_start_;
    bool base64_finished_;
};

}  // namespace gwmail

#endif  // GWMAIL_MIME_WRITER_H
