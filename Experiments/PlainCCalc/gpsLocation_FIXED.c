#include <stdio.h>
#include <stdint.h>
#include "../../lib/q_math.h"

/*
q_math.hを使って緯度経度を10進数の固定小数点（Q23）に変換する．
*/

int main(void) {
    const double goal_longitude = 135.771870; //経度
    const double goal_latitude = 34.800289; //緯度
    
    printf("goal longitude: %ld\n", Q23_FROMF(goal_longitude));
    printf("goal latitude: %ld", Q23_FROMF(goal_latitude));

    return 0;
}