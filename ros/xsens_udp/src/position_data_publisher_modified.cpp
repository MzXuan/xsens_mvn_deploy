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


void parse_header(Header *header, char *buf, int *stamp_sec, int *stamp_nsec)
{
    int hour = 0;
    int min = 0;
    int sec = 0;
    int nanosec = 0;
    float a = 0.0;
    int to_stamp_sec = 0;
    float nano_1000;
    float temp;
    float float_time_code;
    double b = 0.0;
    memcpy(header->ID_String, buf, 6);
    memcpy(&(header->sample_counter), buf+6, 4);
    header->sample_counter = ntohl(header->sample_counter);
    header->datagram_counter = buf[11];
    header->number_of_items = buf[12];
    memcpy(&(header->time_code), buf+12, 4);
    header->time_code = ntohl(header->time_code);
    float_time_code = (float)header->time_code;
    //decoding time_code;
    if (header->time_code <= 99999) { //if time_code < 99.999;
	float_time_code /= 1000;
	//(debug) printf("float_time_code = header->time_code/1000 = %f\n", float_time_code);
	sec = (int)float_time_code;
	//(debug) printf("sec = (int)float_time_code = %d\n", sec);
	nanosec = (int)((float_time_code - sec)*1000);
	//(debug)printf("nanosec = %d\n", nanosec);
		
	//**convert to time stamp**
	*stamp_sec = sec;
	*stamp_nsec = nanosec;
    } else if (header->time_code > 99999 && header->time_code <= 9999999) { //if 99.999 < time_code <= 99:99.999;
	float_time_code = float_time_code/100000;
	min = (int)float_time_code;
	a = (int)((float_time_code - min)*100000);
	float_time_code = (float)a;
	float_time_code /= 1000;
	sec = (int)float_time_code;
	nanosec = (int)((float_time_code - sec)*1000);
	to_stamp_sec = sec + min*60;
	/* wrong logic!!
	   nanosec = (int)((float_time_code - a)*1000); 
	   float_time_code = (float)a;
	   float_time_code /= 100;
	   min = (int)float_time_code;
	   sec = (int)((float_time_code - min)*100);
	   to_stamp_sec = sec + min*60; */
	//**convert to time stamp**
	*stamp_sec = to_stamp_sec;
	*stamp_nsec = nanosec;
    } else if (header->time_code > 9999999 && header->time_code <= 999999999) { //if 99:99.999 < time_code < 99:99:99.999;
	float_time_code /= 10000000;
	hour = (int)float_time_code;
	a = (int)((float_time_code - hour)*10000000);
	float_time_code = (float)a;
	float_time_code = float_time_code/100000;
	min = (int)float_time_code;
	a = (int)((float_time_code - min)*100000);
	float_time_code = (float)a;
	float_time_code /= 1000;
	sec = (int)float_time_code;
	nanosec = (int)((float_time_code - sec)*1000);
	to_stamp_sec = sec + min*60 + hour*60*60;
	/*wrong logic!!
	  nanosec = (int)((float_time_code - a)*1000);
	  float_time_code = (float)a;
	  float_time_code /= 100;
	  min = (int)float_time_code;
	  sec = (int)((float_time_code - min)*100);
	  header->time_code = (int)header->time_code;
	  header->time_code = header->time_code/100;
	  hour = (int)header->time_code;
	  min = (header->time_code - hour)*100;
	  sec = sec + min*60 + hour*60*60; */
	//convert to time stamp
	*stamp_sec = sec;
	*stamp_nsec = nanosec;
    }
    header->character_ID = buf[17];
    memcpy(header->reserved_for_future_use, buf+18, 7);
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


float convertFromRadToDeg(float rad)
{
    float deg = rad*180/PI;
    return deg;
}


void parse_body(char *buf, int *segment_id, float *x, float *y, float *z)
{
    int seg_id = 0;
    float x_p, y_p, z_p = 0.0;
    float q1_r, q2_r, q3_r, q4_r = 0.0;
    memcpy(&seg_id, buf, 4);
    seg_id = ntohl(seg_id);
    x_p = parse_coordinates(x_p, 4, buf);
    //Notice: can't use ntohl() here
    //memcpy(&x_p, buf+4, 4);
    //x_p = ntohl(x_p);
    y_p = parse_coordinates(y_p, 8, buf);
    z_p = parse_coordinates(z_p, 12, buf);
    	
    q1_r = parse_coordinates(q1_r, 16, buf);
    q1_r = convertFromRadToDeg(q1_r);
    q2_r = parse_coordinates(q2_r, 20, buf);
    q2_r = convertFromRadToDeg(q2_r);
    q3_r = parse_coordinates(q3_r, 24, buf);
    q3_r = convertFromRadToDeg(q3_r);
    q4_r = parse_coordinates(q4_r, 28, buf);
    q4_r = convertFromRadToDeg(q4_r);

    *segment_id = seg_id;
    *x = x;
    *y = y;
    *z = z;

}


void handle_udp_msg(int fd)
{
    char *buf = (char*)malloc(1024);
    int read_bytes = 0; 
    int cur_index = 0;
    int count;
    int time_stamp_sec;
    int time_stamp_nsec;
    float x, y, z;
    int segment_id;
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
		parse_header(&header, buf+cur_index, &time_stamp_sec, &time_stamp_nsec);
		cur_index += 24;
		if (strncmp(header.ID_String, "MXTP02", 6) == 0) {
		    //Message type 02 - Pose data (Quaternion)
		    if (read_bytes >= 24+32*header.datagram_counter) {
			int i = 0;
			for (i = 0; i < header.datagram_counter; i++) {
			    parse_body(buf+cur_index+i*32, &segment_id, &x, &y, &z);
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






