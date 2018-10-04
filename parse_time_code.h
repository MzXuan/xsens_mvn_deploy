void parse_time_code(int *buf)
{
	int HH, MM, SS = 0 ; // Time code - 12 byte string formatted as such HH:MM:SS.mmm
	memcpy(&HH, buf, 4);
	HH = ntohl(HH);
	memcpy(&MM, buf+4, 4);
	MM = ntohl(MM);
	memcpy(&SS, buf+8, 4);
	SS = ntohl(SS);

	printf("Time Code: %d:%d:%d\n", HH, MM, SS);

}
