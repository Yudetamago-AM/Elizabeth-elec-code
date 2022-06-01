#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/*
上手くうごかんので適宜MATLAB等で計算
*/

int main(void) {
    long double goal_longitude = 135.771870; //経度
    long double goal_latitude = 34.800289; //緯度

    char buf[1024];
    snprintf(buf, sizeof(buf), "%a", goal_longitude * 60000);
    printf("longitude: %s\n", buf);
    snprintf(buf, sizeof(buf), "%a", goal_latitude * 60000);
    printf("latitude: %s", buf);

    return 0;
}