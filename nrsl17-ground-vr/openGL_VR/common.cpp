#include "common.h"
void encodePacket(unsigned char id, double value, char *buff)
{
	((uint16_t *)buff)[0] = 61423;
	memcpy(&buff[2], &id, 1);
	memcpy(&buff[3], &value, 8);
	std::uint32_t crc = CRC::Calculate(buff, 11, CRC::CRC_32());
	memcpy(&buff[11], &crc, 4);
}
bool decodePacket(char *buff, unsigned char &id, double &value)
{
	if (((uint16_t *)buff)[0] != 61423)
	{
		return false;
	}
	std::uint32_t crc = CRC::Calculate(buff, 11, CRC::CRC_32());
	if (crc != ((uint32_t *)(&buff[11]))[0])
	{
		return false;
	}
	id = (unsigned char)buff[2];
	value = ((double *)(&buff[3]))[0];
	return true;
}
