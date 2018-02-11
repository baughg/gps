// gps.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <stdio.h>
#include <vector>
#include <string>
typedef struct bitfield
{
	unsigned b0 : 1;
	unsigned b1 : 1;
	unsigned b2 : 1;
	unsigned b3 : 1;
	unsigned b4 : 1;
	unsigned b5 : 1;
	unsigned b6 : 1;
	unsigned b7 : 1;
}bitfield;

// YYYYYYYMMMMDDDDDHHHHMMMMMMSSSSSS

typedef struct date_time {
	unsigned sec : 6;
	unsigned min : 6;
	unsigned hrs : 4;
	unsigned day : 5;
	unsigned mth : 4;
	unsigned yrs : 7;
}date_time;

typedef struct date_time_rev {
	unsigned yrs : 7;
	unsigned mth : 4;
	unsigned day : 5;
	unsigned hrs : 4;
	unsigned min : 6;
	unsigned sec : 6;
}date_time_rev;

uint8_t decode_packet(std::vector<uint8_t> &packet);
void decode_packet_bruteforce(std::vector<uint8_t> &buffer);

size_t offset = 0;

int main(int argc, char** argv)
{
	if (argc < 2)
	{
		printf("gps [capture.dat]\n");
		exit(1);
	}

	double PI = 3.14159265359;
	float a = 53.2009* PI / 180.0;
	float b = 6.1111* PI / 180.0;

	unsigned &a_i = *((unsigned*)&a);
	unsigned &b_i = *((unsigned*)&b);

	FILE* dump_file = NULL;
	fopen_s(&dump_file, argv[1], "rb");
	size_t file_size = 0ULL;
	std::vector<uint8_t> buffer(4);

	if (dump_file)
	{
		fseek(dump_file, 0, SEEK_END);
		file_size = ftell(dump_file);
		fseek(dump_file, 0, SEEK_SET);
		buffer.resize(file_size);
		fread(&buffer[0], 1, file_size, dump_file);
		fclose(dump_file);
	}

	std::vector<uint8_t> packet;
	packet.reserve(64);

	int state = -1;
	int payload_size = 0;
	FILE* clean_file = NULL;
	std::string all_filename(argv[1]);
	all_filename.append(".gps");

	fopen_s(&clean_file, all_filename.c_str(), "wb");

	for (size_t b = 0; b < file_size; ++b)
	{
		if (buffer[b] == 0x55 && state == -1)
		{
			state = 0;
			payload_size = 0;
		}
		else if (buffer[b] == 0xaa && state == 0)
		{
			state = 1;
		}
		else if (state == 1)
		{
			state = 2;
		}
		else if (state == 2)
		{
			state = 3;
			payload_size = (int)buffer[b] + 2;
		}
		else if (payload_size > 0)
		{
			payload_size--;
		}
		else if (!payload_size)
		{
			if (packet.size() == 64) {
				uint8_t mask = decode_packet(packet);
				fwrite(&packet[0], 1, packet.size(), clean_file);
				fwrite(&mask, 1, 1, clean_file);
				mask -= packet[51];
				fwrite(&mask, 1, 1, clean_file);
			}
			else
				printf("PACKET SIZE: %llu\n", packet.size());
			/*
			This is still to be confirmed but I believe the 0x30 message carries the GPS module hardware id and firmware version.

55 AA 30 0C XX XX XX XX FW FW FW FW HW HW HW HW CS CS

Note that you need to read version numbers backwards (02 01 00 06 means v6.0.1.2)

HEADER
-------------
BYTE 1-2: message header - always 55 AA
BYTE 3: message id (0x30 for GPS module versions message)
BYTE 4: length of the payload (0x0C or 12 decimal for 0x30 message)

PAYLOAD
--------------
BYTE 5-8" ??? (seems to be always 0)
BYTE 9-12 (FW): firmware version
BYTE 13-16 (HW): hardware id

CHECKSUM
-----------------
BYTE 17-18 (CS): checksum, calculated the same way as for uBlox binary messages*/
			offset += packet.size();
			packet.clear();
			state = 0;
		}

		packet.push_back(buffer[b]);
	}

	fclose(clean_file);
	return 0;
}

unsigned seq_no = 0;

uint8_t decode_packet(std::vector<uint8_t> &buffer)
{
	uint32_t* p_packet = (uint32_t*)&buffer[0];
	uint8_t* p_packet_byte = (uint8_t*)&buffer[0];

	uint8_t mask = 0;

	/*mask[0] = 53rdByte[0] xor 61stByte[4]
	mask[1] = 53rdByte[1] xor 61stByte[5]
	mask[2] = 53rdByte[2] xor 61stByte[6]
	mask[3] = 53rdByte[3] xor 61stByte[7] xor 53rdByte[0];
	mask[4] = 53rdByte[1];
	mask[5] = 53rdByte[2];
	mask[6] = 53rdByte[3];
	mask[7] = 53rdByte[0] xor 61stByte[4];*/

	uint8_t b53, b61;

	bitfield *p_b53, *p_b61, *p_mask;

	p_b53 = (bitfield *)&b53;
	p_b61 = (bitfield *)&b61;
	p_mask = (bitfield *)&mask;

	p_b53->b0 = 1;
	if (*p_packet == 0x3a10aa55)
	{
		b53 = p_packet_byte[52];
		b61 = p_packet_byte[60];

		p_mask->b0 = p_b53->b0 ^ p_b61->b4;
		p_mask->b1 = p_b53->b1 ^ p_b61->b5;
		p_mask->b2 = p_b53->b2 ^ p_b61->b6;
		p_mask->b3 = p_b53->b3 ^ p_b61->b7 ^ p_b53->b0;
		p_mask->b4 = p_b53->b1;
		p_mask->b5 = p_b53->b2;
		p_mask->b6 = p_b53->b3;
		p_mask->b7 = p_b53->b0 ^ p_b61->b4;
	}

	if (mask != buffer[51])
	{
		printf("NO! calc mask = 0x%.2X NOT P[51]=0x%.2X\n", mask, buffer[51]);
		//mask = buffer[51];
	}

	std::vector<uint8_t> decoder(buffer.size());

	for (size_t b = 0; b < decoder.size(); ++b)
	{
		decoder[b] = buffer[b] ^ mask;
	}

	uint32_t * p_decoded = (uint32_t*)&decoder[0];
	date_time &dt = *((date_time*)&p_decoded[1]);
	int &longitude = *((int*)&p_decoded[2]);
	int &latitude = *((int*)&p_decoded[3]);
	unsigned &altitude = *((unsigned*)&p_decoded[4]);
	unsigned &horizontal_accuracy = *((unsigned*)&p_decoded[5]);
	unsigned &veritical_accuracy = *((unsigned*)&p_decoded[6]);
	unsigned &always_zero = *((unsigned*)&p_decoded[7]);
	unsigned & north_velocity = *((unsigned*)&p_decoded[8]);
	unsigned & east_velocity = *((unsigned*)&p_decoded[9]);

	uint8_t number_of_satellites = buffer[52];
	uint16_t sequence_number = *((uint16_t*)&buffer[60]);
	float lat = (float)latitude;
	lat /= 1.0e7;
	float lng = (float)longitude;
	lng /= 1.0e7;

	if (number_of_satellites > 20)
	{
		printf("error!\n");
		decode_packet_bruteforce(buffer);
	}
	
	printf("%05u. mask=0x%.2X, %f, %f [%u] : sats - %u (%u:%u:%u)\n", seq_no++, mask, lat, lng, sequence_number, (unsigned)number_of_satellites, dt.hrs, dt.min, dt.sec);

	return mask;
}


void decode_packet_bruteforce(std::vector<uint8_t> &buffer)
{
	uint32_t* p_packet = (uint32_t*)&buffer[0];
	uint8_t* p_packet_byte = (uint8_t*)&buffer[0];

	uint8_t mask = 0;

	/*mask[0] = 53rdByte[0] xor 61stByte[4]
	mask[1] = 53rdByte[1] xor 61stByte[5]
	mask[2] = 53rdByte[2] xor 61stByte[6]
	mask[3] = 53rdByte[3] xor 61stByte[7] xor 53rdByte[0];
	mask[4] = 53rdByte[1];
	mask[5] = 53rdByte[2];
	mask[6] = 53rdByte[3];
	mask[7] = 53rdByte[0] xor 61stByte[4];*/

	uint8_t b53, b61;

	bitfield *p_b53, *p_b61, *p_mask;

	p_b53 = (bitfield *)&b53;
	p_b61 = (bitfield *)&b61;
	p_mask = (bitfield *)&mask;

	p_b53->b0 = 1;
	if (*p_packet == 0x3a10aa55)
	{
		b53 = p_packet_byte[52];
		b61 = p_packet_byte[60];

		p_mask->b0 = p_b53->b0 ^ p_b61->b4;
		p_mask->b1 = p_b53->b1 ^ p_b61->b5;
		p_mask->b2 = p_b53->b2 ^ p_b61->b6;
		p_mask->b3 = p_b53->b3 ^ p_b61->b7 ^ p_b53->b0;
		p_mask->b4 = p_b53->b1;
		p_mask->b5 = p_b53->b2;
		p_mask->b6 = p_b53->b3;
		p_mask->b7 = p_b53->b0 ^ p_b61->b4;
	}

	std::vector<uint8_t> decoder(buffer.size());

	for (unsigned mask = 0; mask < 256; ++mask) {
		for (size_t b = 0; b < decoder.size(); ++b)
		{
			decoder[b] = buffer[b] ^ (uint8_t)mask;
		}

		uint32_t * p_decoded = (uint32_t*)&decoder[0];
		date_time &dt = *((date_time*)&p_decoded[1]);
		int &longitude = *((int*)&p_decoded[2]);
		int &latitude = *((int*)&p_decoded[3]);
		unsigned &altitude = *((unsigned*)&p_decoded[4]);
		unsigned &horizontal_accuracy = *((unsigned*)&p_decoded[5]);
		unsigned &veritical_accuracy = *((unsigned*)&p_decoded[6]);
		unsigned &always_zero = *((unsigned*)&p_decoded[7]);
		unsigned & north_velocity = *((unsigned*)&p_decoded[8]);
		unsigned & east_velocity = *((unsigned*)&p_decoded[9]);

		uint8_t number_of_satellites = buffer[52];
		uint16_t sequence_number = *((uint16_t*)&buffer[60]);
		float lat = (float)latitude;
		lat /= 1.0e7;
		float lng = (float)longitude;
		lng /= 1.0e7;

		if (dt.yrs == 18)
		{
			printf("[ERROR] %u %05u. mask=0x%.2X, %f, %f [%u] : sats - %u (%u:%u:%u)\n", (unsigned)mask, seq_no, mask, lat, lng, sequence_number, (unsigned)number_of_satellites, dt.hrs, dt.min, dt.sec);
		}
	}
}
