#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "header.h"
#include "parse_body_type_01.h"
#include "parse_body_type_02.h"
#include "parse_body_type_20.h"
#include "parse_body_type_21.h"
#include "parse_body_type_22.h"
#include "parse_body_type_24.h"
#include "parse_body_type_25.h"

#define SERVER_PORT 9763
#define BUFF_LEN 2048

void parse_header(Header *header, char *buf)
{
    memcpy(header->ID_String, buf, 6);
	memcpy(&(header->sample_counter), buf+6, 4);
	header->sample_counter = ntohl(header->sample_counter);
	header->datagram_counter = buf[11];
	header->number_of_items = buf[12];
	memcpy(&(header->time_code), buf+12, 4);
	header->time_code = ntohl(header->time_code);
	header->character_ID = buf[17];
	memcpy(header->reserved_for_future_use, buf+18, 7);
	printf("ID_String = %s\n", header->ID_String);
	printf("sample_counter = %d\n", header->sample_counter);   //字节序转换
	printf("datagram_counter = %d\n", header->datagram_counter);  
	printf("number_of_items = %d\n", header->number_of_items);
	printf("time_code = %d\n", header->time_code);
	printf("character_ID = %d\n", header->character_ID);
}

void handle_udp_msg(int fd)
{
	char *buf = (char*)malloc(BUFF_LEN);
	memset(buf, 0, BUFF_LEN);
	int read_bytes = 0; 
	int cur_index = 0;
	int count;
	struct sockaddr_in client_addr;      // ?sockaddr_in
	socklen_t len = sizeof(client_addr);
	Header header;
	int j = 0;
	while(1)
	{
		count = recvfrom(fd, buf+read_bytes, BUFF_LEN, 0, (struct sockaddr*)&client_addr, &len);
		if(count == -1)
		{
			printf("receieve data fail!\n");
			return;
		}
		read_bytes += count;
		if (read_bytes >= 24) {
			parse_header(&header, buf+cur_index);
			cur_index += 24;
			if (strncmp(header.ID_String, "MXTP01", 6) == 0) {
			//Message type 01 - Pose data (Euler) <- MotionBuilder + Maya
				if (read_bytes >= 24+28*header.datagram_counter) {
					int i = 0;
					printf("Message type 01: Pose data (Euler)\n");
					for (i = 0; i < header.datagram_counter; i++) {
					    parse_body_type_01(buf+cur_index+i*28);
					}
					cur_index += 28*header.datagram_counter;
					int left = read_bytes - cur_index;
					printf("bytes left to parse = %d\n", left);
					if (left > 0)
						memcpy(buf, buf+cur_index, left);
					read_bytes = left;
					cur_index = 0;
				} else {	
					cur_index -= 24; 
				}
				//break;

			} else if (strncmp(header.ID_String, "MXTP02", 6) == 0) {
				
				//Message type 02 - Pose data (Quaternion) <- MVN Analyze/Animate Network Monitor
					if (read_bytes >= 24+32*header.datagram_counter) {
						int i = 0;
						printf("Message type 02: Pose data (Quaternion)\n"); 
						for (i = 0; i < header.datagram_counter; i++) {
							parse_body_type_02(buf+cur_index+i*32);
						}
						cur_index += 32*header.datagram_counter;
						int left = read_bytes - cur_index;
						printf("bytes left to parse = %d\n", left);
						if (left > 0)
							memcpy(buf, buf+cur_index, left);
						read_bytes = left;
						cur_index = 0;
					} else {
						cur_index -= 24;
					}
			} else if (strncmp(header.ID_String, "MXTP03", 6) == 0) {
				//Message type 03 - Pose data (Position Only, MVN Optical marker set 1)

			} else if (strncmp(header.ID_String, "MXTP05", 6) == 0) {	 
				//Message type 05 - Pose data (Unity 3D)

			} else if (strncmp(header.ID_String, "MXTP12", 6) == 0) {
				//Message type 12 - Character information -> meta data
				//(contains name as displayed in MVN Analyze; BodyPack/Awinda-station ID, color of the character)
			
			} else if (strncmp(header.ID_String, "MXTP13", 6) == 0) {
				//Message type 13 - Character information -> scaling information
			} else if (strncmp(header.ID_String, "MXTP20", 6) == 0) {
				//Message type 20 - Joint angle data
					if (read_bytes >= 24+20*header.datagram_counter) {
						int i = 0;
						for (i = 0; i < header.datagram_counter; i++) {
							parse_body_type_20(buf+cur_index+i*20);
						}
						cur_index += 20*header.datagram_counter;
						int left = read_bytes - cur_index;
						printf("bytes left to parse = %d\n", left);
						if (left > 0)
							memcpy(buf, buf+cur_index, left);
						read_bytes = left;
						cur_index = 0;
					} else {
						cur_index -= 24;
					}
			} else if (strncmp(header.ID_String, "MXTP21", 6) == 0) {
				//Message type 21 - Linear Segment Kinematics
					if (read_bytes >= 24+40*header.datagram_counter) {
						int i = 0;
						for (i = 0; i < header.datagram_counter; i++) {
							parse_body_type_21(buf+cur_index+i*40);
						}
						cur_index += 40*header.datagram_counter;
						int left = read_bytes - cur_index;
						printf("bytes left to parse = %d\n", left);
						if (left > 0)
							memcpy(buf, buf+cur_index, left);
						read_bytes = left;
						cur_index = 0;
					} else {
						cur_index -= 24;
					}
			} else if (strncmp(header.ID_String, "MXTP22", 6) == 0) {
				//Message type 22 - Angular Segment Kinematics
					if (read_bytes >= 24+44*header.datagram_counter) {	
						int i = 0;
						for (i = 0; i < header.datagram_counter; i++) {
							parse_body_type_22(buf+cur_index+i*44);
						}
						cur_index += 44*header.datagram_counter;
						int left = read_bytes - cur_index;
						printf("bytes left to parse = %d\n", left);
						if (left > 0)
							memcpy(buf, buf+cur_index, left);
						read_bytes = left;
						cur_index = 0;
					} else {
						cur_index -= 24;
					}
			} else if (strncmp(header.ID_String, "MXTP23", 6) == 0) {
				//Message type 23 - Motion Tracker Kinematics
			
			} else if (strncmp(header.ID_String, "MXTP24", 6) == 0) {
				//Message type 24 - Center of Mass
					if (read_bytes >= 24+12*header.datagram_counter) {
						int i = 0;
						for (i = 0; i < header.datagram_counter; i++) {
							parse_body_type_24(buf+cur_index+i*12);
						}
						cur_index += 12*header.datagram_counter;
						int left = read_bytes - cur_index;
						printf("bytes left to parse = %d\n", left);
						if (left > 0)
							memcpy(buf, buf+cur_index, left);
						read_bytes = left;
						cur_index = 0;
					} else {
						cur_index -= 24;
					}
			} else if (strncmp(header.ID_String, "MXTP25", 6) == 0) {
				//Message type 25 - Time Code
					if (read_bytes >=24+12*header.datagram_counter) {
						int i = 0;
						for (i = 0; i < header.datagram_counter; i++) {
							parse_body_type_25(buf+cur_index+i*12);
						}
						cur_index += 12*header.datagram_counter;
						int left = read_bytes - cur_index;
						printf("bytes left to parse = %d\n", left);
						if (left > 0)
							memcpy(buf, buf+cur_index, left);
						read_bytes = left;
						cur_index = 0;
					} else {
						cur_index -= 24;
					}
			} else {
				printf("Other Type\n");
			}	
		}
				//break;
		
		j ++;
		printf("j = %d\n", j);
		//printf("j = %d\n", j);
		//if (j == 20)
			//break;
	}
	free(buf);
}		
	//	
		//parse_data(buf, &index, &parse_bytes);
		//memset(buf, 0, BUFF_LEN);




/*
      server:
      		socket-->bind-->recvfrom-->sendto-->close
*/

int main(int argc, char* argv[])
{
	int server_fd, ret;
	struct sockaddr_in ser_addr;
	
	server_fd = socket(AF_INET, SOCK_DGRAM, 0); //AF_INET: IPV4; SOCK_DGRAM: UDP
	if(server_fd < 0)
	{
		printf("create socket fail!\n");
		return -1;
	}
	
	memset(&ser_addr, 0, sizeof(ser_addr));
	ser_addr.sin_family = AF_INET;
	ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	ser_addr.sin_port = htons(SERVER_PORT);
	
	ret = bind(server_fd, (struct sockaddr*)&ser_addr, sizeof(ser_addr));
	if(ret < 0)
	{
		printf("socket bind fail!\n");
		return -1;
	}
		
	handle_udp_msg(server_fd);
	
	close(server_fd);




}
	
	
	
	
	








		
