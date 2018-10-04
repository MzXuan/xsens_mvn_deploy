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

void parse_body_type_21(char *buf)
{
	int seg_id;
	int x_p, y_p, z_p = 0;
	int x_gv, y_gv, z_gv = 0;
	int x_ga, y_ga, z_ga = 0;
	memcpy(&seg_id, buf, 4);
	seg_id = ntohl(seg_id);
	memcpy(&x_p, buf+4, 4);
	x_p = ntohl(x_p);
	memcpy(&y_p, buf+8, 4);
	y_p = ntohl(y_p);
	memcpy(&z_p, buf+12, 4);
	z_p = ntohl(z_p);
	memcpy(&x_gv, buf+16, 4);
	x_gv = ntohl(x_gv);
	memcpy(&y_gv, buf+20, 4);
	y_gv = ntohl(y_gv);
	memcpy(&z_gv, buf+24, 4);
	z_gv = ntohl(z_gv);
	memcpy(&x_ga, buf+28, 4);
	x_ga = ntohl(x_ga);
	memcpy(&y_ga, buf+32, 4);
	y_ga = ntohl(y_ga);
	memcpy(&z_ga, buf+36, 4);
	z_ga = ntohl(z_ga);

	printf("Segment ID: %d\n; Segment Position (%d, %d, %d); ", seg_id, x_p, y_p, z_p);
	printf("Segment global velocity (%d, %d, %d); ", x_gv, y_gv, z_gv);
	printf("Segment Global Angular Acceleration (%d, %d, %d); ", x_ga, y_ga, z_ga);
}
