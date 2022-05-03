# CanSat2022
この文書をソフト全体のドキュメントとして扱えるようにしたいなーと思っています．

## はじめに
ここでは，能代宇宙イベントに向けた缶サットのプログラムを開発する．なお，われわれの缶サットの成功基準は下のように定められている．

| 種別 | 基準 |
| ---- | ---- |
| Minimum Success | 機体の損傷無く着地し，パラシュートを切り離す．制御履歴を保存する． |
| Full Success | 正常な動作を開始し，ゴール5m圏内に近づく． |
| Extra Success | ゴール0mまで近づき停止する． |

## 本番の流れ
1. 前日にゴール場所の座標を調べる．
1. 当日投下前に，動作タイマーをセットする．
1. 投下→自動誘導
この中で使うプログラムをここで書いていく．

## 回路について
### 搭載部品
| 役割 | 型番 | 参考になるサイト | 
| --- | --- | --- |
| マイコン | ATmega328P | https://ht-deko.com/arduino/atmega328p.html |
| GNSS | AE-GYSFDMAXB | https://github.com/askn37/GPS_MTK333X |
| 9軸センサー | AE-BNO055-BO | https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code |
| 超音波距離センサー | HC-SR04 | https://akizukidenshi.com/catalog/g/gM-11009/ |
| microSD | AE-microSD-LLCNV | https://akizukidenshi.com/catalog/g/gK-14015/ |
| モータードライバ | TB67H450FNG | https://www.marutsu.co.jp/contents/shop/marutsu/datasheet/TB67H450FNG.pdf |

なお，マイコンと部品間のピンアサインは，PinAssign.hを参照．

## 必要なライブラリ
### モーター制御
がんばって書く，TB67H450FNGを用いたPWM定電流制御．電流値は，基板上の抵抗で決める．
前進，後進，ブレーキ，超信地旋回ができるように．早さはPWMで指定して呼び出す感じにする．
### GNSS情報取得
https://github.com/askn37/GPS_MTK333X
<br>このなんだか便利（シンプル）そうなライブラリが使えそうです．なので，ちょちょっとメイン制御の方に書くのみで大丈夫と思います．
### ログ記録
AE-microSD-LLCNVを使う．がんばって書く．そんなに大変そうじゃ無い．

## メインプログラム
### メイン制御 [Main.ino]
各Seqをそれぞれ呼び出す．
### Landing()
- 着地判定<br>9軸の加速度の大きさ，（GPSの速度の大きさを見る），念のためタイマーも入れておく．
- パラシュートに絡まないようにする（すこし前に進む？）．
- ニクロム線カット<br>PIN_NICHROME(中身はanalog pin 7(PD7))をHIGHにすればオンになるはず．
#### isLanded()
着地判定

### GuideGPS()
- GPSで精度限界まで誘導する<br>上に上げたライブラリを叩いて今の座標とか取得して良い感じに計算して誘導すれば良いはず．

### GuideDIST()
- 超音波距離センサーで0mまで誘導する

われわれはついてるはずなので，カラーコーン±50cmくらいまでGPSで誘導できるはず．その後，超信地旋回をしながら，カラーコーンの位置と距離を認識して，その距離分進む．

### Goal()
ゴール判定をする．ゴールと判定したら，プログラム終了！
GuideDIST()でむしろ明後日の方向へ行ってしまっているようならば，もういちどGPSで誘導して終わりにする．

### タイマー [Main.ino]
あらかじめ設定してあるタイマー（int Timer）(ms)と，millis()を比較する．
（millis()はプログラム開始からの時間をmsで返す）（http://www.musashinodenpa.com/arduino/ref/index.php?f=0&pos=2524）

### isMoving()
うごいてたらtrue，止まってたらfalseを返す
止まっていることは，スタックしていることを直接意味するわけではない．

### Distance()
超音波距離センサーをつかって，目の前の物体までの距離をcmで返す．
（一応，九軸センサー内の温度センサーをつかって音速を校正しているが，逆に安定性を損なうようだったら，→board tempなのか…ならばつかえないのか）20℃1気圧とかで計算しておく．

## GPS座標取得プログラム
前日にゴールのカラーコーンの座標を調べる時間があるはずなので，そのときにGPSの座標を取得するためのプログラム．
上に挙げたライブラリでただ単に座標を取得すれば良いと思う．