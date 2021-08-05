/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __rs_pointcloud_t_hpp__
#define __rs_pointcloud_t_hpp__



class rs_pointcloud_t
{
    public:
        double     pointlist[5001][3];

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "rs_pointcloud_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int rs_pointcloud_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int rs_pointcloud_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int rs_pointcloud_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t rs_pointcloud_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* rs_pointcloud_t::getTypeName()
{
    return "rs_pointcloud_t";
}

int rs_pointcloud_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    for (int a0 = 0; a0 < 5001; a0++) {
        tlen = __double_encode_array(buf, offset + pos, maxlen - pos, &this->pointlist[a0][0], 3);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int rs_pointcloud_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    for (int a0 = 0; a0 < 5001; a0++) {
        tlen = __double_decode_array(buf, offset + pos, maxlen - pos, &this->pointlist[a0][0], 3);
        if(tlen < 0) return tlen; else pos += tlen;
    }

    return pos;
}

int rs_pointcloud_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += 5001 * __double_encoded_array_size(NULL, 3);
    return enc_size;
}

uint64_t rs_pointcloud_t::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x960bbc9070e3509eLL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
