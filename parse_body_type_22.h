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

void parse_body_type_22(char *buf)
{
	int seg_id = 0;	
	int q1_r, q2_r,q3_r,q4_r = 0;
 	int x_av, y_av, z_av = 0; //Global Angular Velocity
	int x_aa, y_aa, z_aa = 0; //Global Angular Acceleration

	memcpy(&seg_id, buf, 4);
	seg_id = ntohl(seg_id);
	memcpy(&q1_r, buf+4, 4);
	q1_r = ntohl(q1_r);
	memcpy(&q2_r, buf+8, 4);
	q2_r = ntohl(q2_r);
	memcpy(&q3_r, buf+12, 4);
	q3_r = ntohl(q3_r);
	memcpy(&q4_r, buf+16, 4);
	q4_r = ntohl(q4_r);
	memcpy(&x_av, buf+20, 4);
	x_av = ntohl(x_av);
	memcpy(&y_av, buf+24, 4);
	y_av = ntohl(y_av);
	memcpy(&z_av, buf+28, 4);
	z_av = ntohl(z_av);
	memcpy(&x_aa, buf+32, 4);
	x_aa = ntohl(x_aa);
	memcpy(&y_aa, buf+36, 4);
	y_aa = ntohl(y_aa);
	memcpy(&z_aa, buf+40, 4);
	z_aa = ntohl(z_aa);	

	printf("Segment ID: %d\n", seg_id);
	printf("q1 rotation(re): %d, q2 rotation(i): %d, q3 rotation(j): %d, q4 rotation(k): %d\n.", q1_r, q2_r, q3_r, q4_r);
	printf("Segment Global Angular Velocity: (%d, %d, %d)\n", x_av, y_av, z_av);
	printf("Segment Global Angular Acceleration: (%d, %d, %d)\n", x_aa, y_aa, z_aa);
}


