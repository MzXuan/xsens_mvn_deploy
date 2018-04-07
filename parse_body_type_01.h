void parse_body_type_01(char *buf)
{
    int seg_id = 0;
    float x_p, y_p, z_p = 0.0;
    float x_r, y_r, z_r = 0.0;
    memcpy(&seg_id, buf, 4);
    seg_id = ntohl(seg_id);



    //memcpy(&x_p, buf+4, 4);
    //x_p = (float)ntohl(x_p);

	unsigned char byte_array[4];
 	memcpy(&byte_array, buf+4, sizeof(x_p));
	  *((unsigned char*)(&x_p) + 3) = byte_array[0];
	  *((unsigned char*)(&x_p) + 2) = byte_array[1];
	  *((unsigned char*)(&x_p) + 1) = byte_array[2];
	  *((unsigned char*)(&x_p) + 0) = byte_array[3];
	x_p /= 100;



    memcpy(&y_p, buf+8, 4);
    y_p = (float)ntohl(y_p);
    memcpy(&z_p, buf+12, 4);
    z_p = (float)ntohl(z_p);
    memcpy(&x_r, buf+16, 4);
    x_r = (float)ntohl(x_r);
    memcpy(&y_r, buf+20, 4);
    y_r = (float)ntohl(y_r);
    memcpy(&z_r, buf+24, 4);
    z_r = (float)ntohl(z_r);

        printf("Segment ID:%d\n ", seg_id);
	printf("Segment Position: (%f, %f, %f)\n", x_p, y_p, z_p);
	printf("Segment Rotation: (%f, %f, %f)\n", x_r, y_r, z_r);

 }


