/*
われわれのCanSatに搭載したGPSモジュールは，下記ライブラリの利用で緯度経度を取得でき，その値は1LSB = 1/600000度となっている．
https://github.com/askn37/GPS_MTK333X
これは，ATmega 328Pがdouble型を扱えない対策であるが，このため，そのまま計算したのでは，方位および距離は求められない．
そのため，この計算を行うためのプログラムとして，本プログラムを作成した．
*/

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

/*
方位を計算する
引数: 現在の緯度(lat)経度(lon)と，ゴールの緯度経度をとる．
戻り値:　北を0度として右（東）は正，左（西）は負で最大PIの値を返す．
たとえば，ちょうど東にゴールがある際，1.57...を返す．
*/
float orientation(int32_t dx, int32_t dy) {
    //printf("atan2: %f", atan2(dy, dx));
    if (dx == 0 && dy == 0) return 0;
    else return atan2(dx, dy); //あえてdx,dyの順
}

long distance(int32_t dx, int32_t dy) {
    return sqrt(dx * dx + dy * dy);
}
//参考
//https://nowokay.hatenablog.com/entry/20120604/1338773843
//https://dora.bk.tsukuba.ac.jp/~takeuchi/?%E3%83%97%E3%83%AD%E3%82%B0%E3%83%A9%E3%83%9F%E3%83%B3%E3%82%B0%2F%E5%B9%B3%E6%96%B9%E6%A0%B9%E3%82%92%E4%BD%BF%E3%82%8F%E3%81%9A%E3%81%AB%E8%B7%9D%E9%9B%A2%E3%82%92%E6%B1%82%E3%82%81%E3%82%8B
int32_t approx_distance( int32_t dx, int32_t dy )
{
   uint32_t min, max, approx;

   if ( dx < 0 ) dx = -dx;
   if ( dy < 0 ) dy = -dy;

   if ( dx < dy )
   {
      min = dx;
      max = dy;
   } else {
      min = dy;
      max = dx;
   }

   approx = ( max * 983 ) + ( min * 407 );
   if ( max < ( min << 4 ))
      approx -= ( max * 40 );

   // add 512 for proper rounding
   return (( approx + 512 ) >> 10 );
} 

int main(void) {
    int32_t lat_goal = 20881351;
    int32_t lon_goal = 81461532;
    //int32_t lat_now = 20881441;
    //int32_t lon_now = 81461725;
    int32_t lat_now = 20881124;
    int32_t lon_now = 81461538;

    int32_t dx = ((lon_goal - lon_now) * 153);//0.000001度で0.92m(京田辺)，0.85m(能代)より，単位メートル
    int32_t dy = ((lat_goal - lat_now) * 185);//0.000001度で0.111m(111)より0.1
    printf("dx: %ld\n", dx);
    printf("dy: %ld\n", dy);

    /*
    距離を計算する
    戻り値はcm単位
    参考:
    https://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/bl2stf.html
    */
    printf("distance: %ld(cm)\n", (approx_distance(dx, dy) / 10));

    //方位を計算する
    printf("orientation: %f(rad)\n", orientation(dx, dy));


    /*
    printf("HEX:\n");
    printf("lat_goal: \t0x%x\n", lat_goal);
    printf("lon_goal: \t0x%x\n", lon_goal);
    printf("lat_now: \t0x%x\n", lat_now);
    printf("lon_now: \t0x%x\n", lon_now);

    printf("DEC:\n");
    printf("lat_goal: \t%d\n", lat_goal);
    printf("lon_goal: \t%d\n", lon_goal);
    printf("lat_now: \t%d\n", lat_now);
    printf("lon_now: \t%d\n", lon_now);


    printf("temp HEX: 0x%x\n", temp);
    printf("temp DEC: %d", temp);
    */
}