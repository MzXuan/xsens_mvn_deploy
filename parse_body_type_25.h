/**
void parse_body_type_01(char *buf)
{
    int seg_id = 0;
    int x_p, y_p, z_p = 0;
    int x_r, y_r, z_r = 0;
    memcpy(&seg_id, buf, 4);
    seg_id = ntohl(seg_id);
    memcpy(&x_p, buf+4, 4);
    x_p = ntohl(x_p);
    memcpy(&y_p, buf+8, 4);
    y_p = ntohl(y_p);
    memcpy(&z_p, buf+12, 4);
    z_p = ntohl(z_p);
    memcpy(&x_r, buf+16, 4);
    x_r = ntohl(x_r);
    memcpy(&y_r, buf+20, 4);
    y_r = ntohl(y_r);
    memcpy(&z_r, buf+24, 4);
    z_r = ntohl(z_r);

        printf("%d %d %d %d %d %d %d \n", seg_id, x_p, y_p, z_p, x_r, y_r, z_r);
}
**/

void parse_body_type_25(char *buf)
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
