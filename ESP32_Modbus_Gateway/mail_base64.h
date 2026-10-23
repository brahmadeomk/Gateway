// Streaming base64 encoder for MIME bodies.
//
// Encodes in chunks rather than assembling the whole encoded string in one
// allocation, so a large attachment never becomes one heap-sized String on
// top of the message body.
//
// The encoder holds only a 3-byte carry between calls, so a caller can push an
// attachment through in arbitrary slices and write each output chunk straight to the
// socket.

#ifndef GWMAIL_BASE64_H
#define GWMAIL_BASE64_H

#include <stddef.h>
#include <stdint.h>

namespace gwmail {

// Encoded length of `input_len` bytes, excluding line breaks and NUL.
size_t Base64EncodedLength(size_t input_len);

// One-shot encode of a NUL-terminated string, NUL-terminating the result.
// Returns the number of characters written, or 0 if the buffer is too small.
//
// For short strings only - AUTH LOGIN sends its username and password this way
// (RFC 4954). Callers must treat the output as secret when the input was.
size_t Base64EncodeString(const char* text, char* out, size_t out_len);

class Base64Encoder {
  public:
    Base64Encoder();

    void Reset();

    // Encode `len` bytes into `out`, returning the number of characters
    // written. Requires room for at least ((len + 2) / 3) * 4 characters.
    // Leftover bytes are carried into the next call.
    size_t Update(const uint8_t* data, size_t len, char* out, size_t out_len);

    // Emit the padded final group, if any. Writes at most 4 characters.
    size_t Final(char* out, size_t out_len);

  private:
    uint8_t carry_[3];
    size_t carry_len_;
};

}  // namespace gwmail

#endif  // GWMAIL_BASE64_H
