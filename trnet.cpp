#include "trnet.h"
#include <cstring>

TRNet::TRNet() : parse_data_len_(0)
{
}

uint8_t TRNet::CalCheckSum(const uint8_t *data, uint16_t len)
{
    uint8_t checksum = 0;
    std::size_t i = 0;

    for (i = 0; i < len; i++)
    {
        checksum += *data++;
    }

    return checksum;
}

const TRData *TRNet::Unpack(const uint8_t *data, uint32_t len)
{
    if (data == nullptr || len < EXTRA_LEN)
    {
        return nullptr;
    }
    const uint8_t *p = data;
    uint32_t code = *(uint32_t *)data;
    if (code != LEADING_CODE)
    {
        return nullptr;
    }
    p += 8;
    uint16_t data_len = *(uint16_t *)p;

    if (data_len > (len - EXTRA_LEN))
    {
        return nullptr;
    }

    p += 2;

    uint8_t checksum = CalCheckSum(data + 4, 6 + data_len);

    p += data_len;

    if (checksum == *p)
    {
        p = data;
        p += 4;
        tr_data_.device_address = *p++;
        tr_data_.pack_id = *p++;
        tr_data_.chunk_offset = *(uint16_t *)p;
        p += 2;
        if (tr_data_.data.size() < data_len)
            tr_data_.data.resize(data_len);
        p += 2;
        std::memcpy(tr_data_.data.data(), p, data_len);
        parse_data_len_ = data_len + EXTRA_LEN;
        return &tr_data_;
    }

    return nullptr;
}


bool TRNet::Pack(const TRData &in, std::vector<uint8_t> &out)
{
    out.resize(EXTRA_LEN + in.data.size());
    uint8_t *p = out.data();
    *(uint32_t *)p = LEADING_CODE;
    p += 4;
    *p++ = in.device_address;
    *p++ = in.pack_id;
    *(uint16_t *)p = in.chunk_offset;
    p += 2;
    *(uint16_t *)p = in.data.size();
    p += 2;
    std::memcpy(p, in.data.data(), in.data.size());
    uint8_t checksum = CalCheckSum(out.data() + 4, out.size() - 5);
    out.back() = checksum;
    return true;
}

bool TRNet::FindLeadingCode(const uint8_t *buff)
{
    uint32_t code = *(uint32_t *)buff;
    return (code == LEADING_CODE);
}
