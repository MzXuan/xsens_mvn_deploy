
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

typedef struct header {
	char ID_String[6];
	int sample_counter;
	char datagram_counter;
	char number_of_items;
	int time_code;
	char character_ID;
	char reserved_for_future_use[7];
}Header;

void parse_header(Header *header, int *time_stamp_sec, int *time_stamp_nsec)
{
    while(1) {
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
   
	scanf("%d", &(header->time_code));
	// header->time_code = ntohl(header->time_code);
	float_time_code = (float)header->time_code;
	//decoding time_code;
	if (header->time_code <= 99999) { //if time_code < 99.999;
	    float_time_code /= 1000;

	    sec = (int)float_time_code;

	    nanosec = (int)((float_time_code - sec)*1000);
	    to_stamp_sec = sec;
		
	    //**convert to time stamp**
	    *time_stamp_sec = to_stamp_sec;
	    *time_stamp_nsec = nanosec;
	} else if (header->time_code > 99999 && header->time_code <= 9999999) { //if 99.999 < time_code <= 99:99.999;
	    float_time_code = float_time_code/100000;
	    min = (int)float_time_code;
	    a = (int)((float_time_code - min)*100000);
	    float_time_code = (float)a;
	    float_time_code /= 1000;
	    sec = (int)float_time_code;
	    nanosec = (int)((float_time_code - sec)*1000);
	    to_stamp_sec = sec + min*100;
	    /* wrong logic!!
	       nanosec = (int)((float_time_code - a)*1000); 
	       float_time_code = (float)a;
	       float_time_code /= 100;
	       min = (int)float_time_code;
	       sec = (int)((float_time_code - min)*100);
	       to_stamp_sec = sec + min*60; */
	    //**convert to time stamp**
	    *time_stamp_sec = to_stamp_sec;
	    *time_stamp_nsec = nanosec;
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
	    to_stamp_sec = sec + min*100 + hour*100*100;
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
	    *time_stamp_sec = to_stamp_sec;
	    *time_stamp_nsec = nanosec;
	}
	
	

	printf("time_code = %d\n", (int)header->time_code);
	printf("time = %d:%d:%d.%d\n", hour, min, sec, nanosec);
	printf("time stamp for rosmsg: %d.%d\n", *time_stamp_sec, *time_stamp_nsec);
    }
}


int main(int argc, char *arcv[])
{
    Header header;
    int time_stamp_sec;
    int time_stamp_nsec;
    parse_header(&header, &time_stamp_sec, &time_stamp_nsec);
    return 0;
}
