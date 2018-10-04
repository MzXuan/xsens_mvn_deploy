/**
void parse_body(char *buf)
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

void parse_body_type_20(char *buf)
{
	int point_id_parent, point_id_child = 0;
	int rotation_around_x, rotation_around_y, rotation_around_z = 0;
	memcpy(&point_id_parent, buf, 4);
	point_id_parent = ntohl(point_id_parent);
	memcpy(&point_id_child, buf+4, 4);
	point_id_child = ntohl(point_id_child);
	memcpy(&rotation_around_x, buf+8, 4);
	rotation_around_x = ntohl(rotation_around_x);
	memcpy(&rotation_around_y, buf+12, 4);
	rotation_around_y = ntohl(rotation_around_y);
	memcpy(&rotation_around_z, buf+16, 4);
	rotation_around_z = ntohl(rotation_around_z);

	printf("Point ID of Parent Segment Connection: %d\n", point_id_parent);
	printf("Point ID of Child Segment Connection: %d\n", point_id_child);
	printf("Floating Point Rotation around Segment (%d, %d, %d)", rotation_around_x, rotation_around_y, rotation_around_z);
}
