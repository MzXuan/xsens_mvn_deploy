#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "header.h"

#define SERVER_PORT 9763
#define BUFF_LEN 1024

#define PI 3.141592


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


float parse_coordinates(float coordinate, int count, char *buf)
{
	//test endian

	unsigned char byte_array[4];
	memcpy(&byte_array, buf+count, sizeof(coordinate));
	*((unsigned char*)(&coordinate) + 3) = byte_array[0];
	*((unsigned char*)(&coordinate) + 2) = byte_array[1];
	*((unsigned char*)(&coordinate) + 1) = byte_array[2];
	*((unsigned char*)(&coordinate) + 0) = byte_array[3];
	//coordinate /= 100; (Only need this in Type 01)
	return coordinate;
}

void convertFromYupToZup(float *x, float *y, float *z)
{
	float x1 = *x;
	float y1 = *y;
	float z1 = *z;
	
	*x = z1;
	*y = x1;
	*z = y1;
}

float convertFromRadToDeg(float rad)
{
	float deg = rad*180/PI;
	return deg;
}

void parse_body(char *buf)
{
	int seg_id = 0;
	float x_p, y_p, z_p = 0.0;
	float q1_r, q2_r, q3_r, q4_r = 0.0;
	memcpy(&seg_id, buf, 4);
	seg_id = ntohl(seg_id);
	x_p = parse_coordinates(x_p, 4, buf);
	//memcpy(&x_p, buf+4, 4);
	//x_p = ntohl(x_p);
	y_p = parse_coordinates(y_p, 8, buf);
	//memcpy(&y_p, buf+8, 4); 	
	//y_p = ntohl(y_p);
	z_p = parse_coordinates(z_p, 12, buf);
	//memcpy(&z_p, buf+12, 4); 	
	//z_p = ntohl(z_p);
	//convertFromYupToZup(&x_p, &y_p, &z_p);
	
	q1_r = parse_coordinates(q1_r, 16, buf);
	q1_r = convertFromRadToDeg(q1_r);
	//memcpy(&x_r, buf+16, 4); 	
	//x_r = ntohl(x_r);
	q2_r = parse_coordinates(q2_r, 20, buf);
	q2_r = convertFromRadToDeg(q2_r);
	//memcpy(&y_r, buf+20, 4); 	
	//y_r = ntohl(y_r);
	q3_r = parse_coordinates(q3_r, 24, buf);
	q3_r = convertFromRadToDeg(q3_r);
	//memcpy(&z_r, buf+24, 4); 	
	//z_r = ntohl(z_r);ll
	q4_r = parse_coordinates(q4_r, 28, buf);
	q4_r = convertFromRadToDeg(q4_r);
	
        printf("Segment ID: %d\n", seg_id);
	printf("Segment Position: (%f, %f, %f)\n", x_p, y_p, z_p);
	printf("Segment Rotation Quaternion: (re:%f, i:%f, j:%f, k:%f)\n", q1_r, q2_r, q3_r, q4_r);
}




void handle_udp_msg(int fd)
{
	char *buf = (char*)malloc(1024);
	int read_bytes = 0; 
	int cur_index = 0;
	int count;
	struct sockaddr_in client_addr;
	socklen_t len = sizeof(client_addr);
	Header header;
	int j = 0;
	while(1)
	{
		memset(buf, 0, BUFF_LEN);
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
			if (strncmp(header.ID_String, "MXTP02", 6) == 0) {
			//Message type 02 - Pose data (Quaternion)
				if (read_bytes >= 24+32*header.datagram_counter) {
					int i = 0;
					printf("Message type 02: Pose data (Quaternion)\n"); 
					for (i = 0; i < header.datagram_counter; i++) {
						parse_body(buf+cur_index+i*32);
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
			} else {
				//TODO
				printf("other type\n");
			}
			//break;
		}		
		j ++;
		printf("j = %d\n", j);
		//if (j == 2)
		    //break;
		//parse_data(buf, &index, &parse_bytes);
		//memset(buf, 0, BUFF_LEN);
	}
	free(buf);
}


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
	
	
	
	
	








		
