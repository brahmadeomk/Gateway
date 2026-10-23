#include "mail_base64.h"

namespace gwmail {
namespace {

const char kAlphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

void EncodeTriple(const uint8_t* in, char* out) {
    const uint32_t group = (static_cast<uint32_t>(in[0]) << 16) |
                           (static_cast<uint32_t>(in[1]) << 8) |
                           static_cast<uint32_t>(in[2]);
    out[0] = kAlphabet[(group >> 18) & 0x3F];
    out[1] = kAlphabet[(group >> 12) & 0x3F];
    out[2] = kAlphabet[(group >> 6) & 0x3F];
    out[3] = kAlphabet[group & 0x3F];
}

}  // namespace

size_t Base64EncodedLength(size_t input_len) {
    return ((input_len + 2) / 3) * 4;
}

size_t Base64EncodeString(const char* text, char* out, size_t out_len) {
    if (text == NULL || out == NULL) {
        return 0;
    }
    size_t len = 0;
    while (text[len] != '\0') {
        len++;
    }
    const size_t needed = Base64EncodedLength(len);
    if (out_len < needed + 1) {
        return 0;
    }

    Base64Encoder encoder;
    size_t written = encoder.Update(reinterpret_cast<const uint8_t*>(text), len,
                                    out, out_len - 1);
    written += encoder.Final(&out[written], out_len - 1 - written);
    out[written] = '\0';
    return written;
}

Base64Encoder::Base64Encoder() : carry_len_(0) {
    carry_[0] = 0;
    carry_[1] = 0;
    carry_[2] = 0;
}

void Base64Encoder::Reset() { carry_len_ = 0; }

size_t Base64Encoder::Update(const uint8_t* data, size_t len, char* out,
                             size_t out_len) {
    if (data == NULL || out == NULL) {
        return 0;
    }
    // Refuse outright rather than encoding a prefix. Consuming input that never
    // reaches the output would silently corrupt the attachment, and the caller
    // has no way to notice from the return value alone.
    if (out_len < Base64EncodedLength(carry_len_ + len)) {
        return 0;
    }
    size_t written = 0;
    size_t consumed = 0;

    // Top up a partial group left by the previous call before starting fresh
    // triples, so the output is identical regardless of how the caller sliced
    // the input.
    while (carry_len_ > 0 && consumed < len) {
        carry_[carry_len_++] = data[consumed++];
        if (carry_len_ == 3) {
            if (written + 4 > out_len) {
                return written;
            }
            EncodeTriple(carry_, &out[written]);
            written += 4;
            carry_len_ = 0;
        }
    }

    while (len - consumed >= 3) {
        if (written + 4 > out_len) {
            return written;
        }
        EncodeTriple(&data[consumed], &out[written]);
        written += 4;
        consumed += 3;
    }

    while (consumed < len) {
        carry_[carry_len_++] = data[consumed++];
    }
    return written;
}

size_t Base64Encoder::Final(char* out, size_t out_len) {
    if (carry_len_ == 0 || out == NULL || out_len < 4) {
        return 0;
    }

    uint8_t group[3] = {0, 0, 0};
    for (size_t i = 0; i < carry_len_; i++) {
        group[i] = carry_[i];
    }
    EncodeTriple(group, out);

    // One leftover byte yields two significant characters, two yield three.
    if (carry_len_ == 1) {
        out[2] = '=';
        out[3] = '=';
    } else {
        out[3] = '=';
    }
    carry_len_ = 0;
    return 4;
}

}  // namespace gwmail
