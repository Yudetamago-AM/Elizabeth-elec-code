#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/*
これでも十分精度出てるのでヨシとする．
*/

int main(void) {
    double goal_longitude = 135.771870; //経度
    double goal_latitude = 34.800289; //緯度

    printf("longitude: %f\n", goal_longitude * 600000);
    printf("latitude: %f", goal_latitude * 600000);

    return 0;
}