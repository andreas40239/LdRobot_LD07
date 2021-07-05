#ifndef _TR_NET_H
#define _TR_NET_H

#include <stdint.h>
#include <vector>

#define THIS_DEVICE_ADDREESS 0x01 /*Device address*/

struct TRData
{
	uint8_t device_address;
	uint8_t pack_id;
	uint16_t chunk_offset;
	std::vector<uint8_t> data;
};


class TRNet
{
public:
	TRNet();
	const TRData *Unpack(const uint8_t *data, uint32_t len);
	bool Pack(const TRData &in, std::vector<uint8_t> &out);
	bool FindLeadingCode(const uint8_t *buff); //make sure buffer size bigger than 4
	uint32_t GetParseDataLen() { return parse_data_len_; }

protected:
	TRData tr_data_;

private:
	const uint32_t LEADING_CODE = 0xAAAAAAAA;
	const uint32_t HEADER_LEN = 4; //device_address , pack_id , chunk_offset len
	const uint32_t EXTRA_LEN = 11;
	uint32_t parse_data_len_;
	uint8_t CalCheckSum(const uint8_t *data, uint16_t len);

};

#endif // _TR_NET_H
