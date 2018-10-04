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




void parse_body_type_24(char *buf)
{
	int x, y, z; //x, y, z center of mass position
	memcpy(&x, buf, 4);
	x = ntohl(x);
	memcpy(&y, buf+4, 4);
	y = ntohl(y);
	memcpy(&z, buf+8, 4);
	z = ntohl(z);

	printf("Center of Mass: (%d, %d, %d)\n", x, y, z);
}
