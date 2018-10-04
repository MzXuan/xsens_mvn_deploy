void parse_body_type_02(char *buf)
{
    int seg_id = 0;
    int x_p, y_p, z_p = 0;
    int q1_r, q2_r, q3_r, q4_r = 0;
    memcpy(&seg_id, buf, 4);
    seg_id = ntohl(seg_id);
    memcpy(&x_p, buf+4, 4);
    x_p = ntohl(x_p);
    memcpy(&y_p, buf+8, 4);
    y_p = ntohl(y_p);
    memcpy(&z_p, buf+12, 4);
    z_p = ntohl(z_p);
    memcpy(&q1_r, buf+16, 4);
    q1_r = ntohl(q1_r);
    memcpy(&q2_r, buf+20, 4);
    q2_r = ntohl(q2_r);
    memcpy(&q3_r, buf+24, 4);
    q3_r = ntohl(q3_r);
    memcpy(&q4_r, buf+28, 4);
    q4_r = ntohl(q4_r);

        //printf("%d %d %d %d %d %d %d %d\n", seg_id, x_p, y_p, z_p, q1_r, q2_r, q3_r, q4_r);
	printf("Segment ID: %d\n", seg_id);
	printf("Segment Position: (%d, %d, %d)\n", x_p, y_p, z_p);
	printf("Segment Rotation Quaternion: (re:%d, i:%d, j:%d, k:%d)", q1_r, q2_r, q3_r, q4_r);
}


