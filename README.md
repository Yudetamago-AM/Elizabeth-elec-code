# CanSat2022
## はじめに
ここでは，能代宇宙イベントに向けた缶サットのプログラムを開発する．なお，われわれの缶サットの成功基準は下のように定められている．

| 種別 | 基準 |
| ---- | ---- |
| Minimum Success | 機体の損傷無く着地し，パラシュートを切り離す．制御履歴を保存する． |
| Full Success | 正常な動作を開始し，ゴール5m圏内に近づく． |
| Extra Success | ゴール0mまで近づき停止する． |

## 回路について
### 搭載部品
| 役割 | 型番 | データシートなど | 
| --- | --- | --- |
| マイコン | ATmega328P | https://akizukidenshi.com/catalog/g/gI-12774/ |
| GNSS | AE-GYSFDMAXB | https://akizukidenshi.com/catalog/g/gK-09991/ |
| 9軸センサー | AE-BNO055-BO | https://github.com/adafruit/Adafruit_BNO055 |
| 超音波距離センサー | HC-SR04 | https://akizukidenshi.com/catalog/g/gM-11009/ |
| microSD | AE-microSD-LLCNV | https://akizukidenshi.com/catalog/g/gK-14015/ |
| モータードライバ | TB67H450FNG | https://www.marutsu.co.jp/contents/shop/marutsu/datasheet/TB67H450FNG.pdf |

なお，マイコンと部品間のピンアサインは，PinAssign.hを参照．

## 必要なライブラリ
### モーター制御
がんばって書く，TB67H450FNGを用いたPWM定電流制御．電流値は，基板上の抵抗で決める．
### GNSS情報取得
https://github.com/askn37/GPS_MTK333X
<br>このなんだか便利（シンプル）なライブラリが使えそうです．なので，ちょちょっとメイン制御の方に書くのみで大丈夫と思います．
### ログ記録
AE-microSD-LLCNVを使う．がんばって書く．そんなに大変そうじゃ無い．

## メインプログラム
これは，メインプログラム（Main.ino）の中に関数として書いておき，適宜呼び出す物である．
### メイン制御
各Seqをそれぞれ呼び出す．
### 着地時(seq1)
やりたいこと
- 着地判定<br>9軸の加速度の大きさ，GPSの速度の大きさを見る，念のためタイマーも入れておく．
- パラシュートに絡まないように，すこし前に進む．
- ニクロム線カット<br>analog pin 7(PD7)をHIGHにすればオンになるはず．
### 大まかな誘導(seq2)
### 精密誘導(seq3)
### ゴール到着時(seq4)

### タイマー
### スタック判定
### 


## 本番の流れ
1. 前日にゴール場所の座標を調べる．
1. 当日投下前に，動作タイマーをセットする．
1. 投下→自動誘導
