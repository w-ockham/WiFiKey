# WiFiKey
## はじめに
WiFiを使ってリモートからリグのキーイングを行う実験用システムです。
***
## ハードウェア  
エレキー・縦振電鍵・バグキー等にも対応できるようにするため入力側はHigh/Lowのレベル入力とします。キー側は入力されたレベルをWiFi経由でパケットにして送信します。リグ側では送られてきたパケットに格納されたレベルでフォトカプラを制御しリグのキーイングを行います。

システムは[ESP32](https://akizukidenshi.com/catalog/g/gM-15674/)を使って実装しています。キー側は`GPIO 14`を使ってキー入力を、リグ側は`GPIO 27`を使ってフォトカプラ[PC817](https://akizukidenshi.com/catalog/g/gI-13765/)を制御しています。キーイングやWiFi接続確認用にLEDを`GPIO 26`に接続しています。GPIOの入力側は全てESP32内部でプルアップしています。また出力側には電流制限用に100Ωの抵抗を入れています。

またキーイング確認用のLED出力と動作設定用の入力にGPIOを割り当てています。

| GPIO | 入出力 | 機能 |
|:-----|:-------|:-----|
| 14 | 入力（内部プルアップ) | キー入力 |
| 26 | 出力 | LED出力 |
| 27 | 出力 | フォトカプラ用出力|
| 25 | 入力(内部プルアップ) | WiFiモード |
| 12 | 入力(内部プルアップ) | パケットタイプ |


![ボード完成図](images/ESP32.jpg)

***

## 動作モードの切り替え
### クライアント・サーバ
起動時にキー入力(`GPIO 14`)のレベルをみて動作モードを決めています。サーバ（リグ側）は`GPIO 14`を`L`に固定しています。サーバは後述するスタンドアロンモードの場合はWiFiのアクセスポイントとしても機能します。

起動時にキー入力(`GPIO 14`)が`H`になっている場合はクライアント(キー側)として動作します。

### WiFiモード
起動時に`GPIO 25`が`H`の場合はサーバーは既設のWiFIアクセスポイントへの接続を行います。SSID/パスワードはコード中に埋め込んでいます。サーバのアドレスは変数`server`のアドレスになります（デフォルト`192.168.1.192`）
クライアント(キー側)はアクセスポイントからDHCPで割り当てられたアドレスとなります。

`GPIO 25`が`L`の場合はサーバはスタンドアロンのWiFiのアクセスポイントとして動作します。サーバのアドレスは変数`ap_server`のアドレスになります(デフォルト`192.168.4.1`)
クライアント（キー側）は変数`client`のアドレス(デフォルト`192.168.4.2`となります。

いずれのモードも、電源投入後アクセスポイントへの接続が完了するまでの間はLEDが点灯したままになります（WiFI接続が切れた場合も再度接続するまでの間はLEDが点灯したままになります）。

### パケットタイプ
キー入力の立ち下がり・立ち上がりのエッジでUDPパケットを送出するエッジタイプ(起動時`GPIO 12`が`L`)と、キー入力の立ち上がりのタイミングでキーダウンされていた時間と送出時刻をUDPパケットで送出する時刻タイプ(起動時`GPIO 12`が`H`)があります。

[エッジタイプ](https://youtu.be/C8p-kPGs3-I)はレイテンシの少ないキーイングが可能です。しかしアクセスポイントを経由して接続するとパケットの到着時刻にジッタが生じてしまいサーバ側で符号を再現することができません。スタンドアローンモードでサーバにクライアントを直接繋ぐ場合にお使いください。

[時刻タイプ](https://youtu.be/Xg9ygcKnHZg)はキーダウンの時間と次の符号までの時刻を保存するためアクセスポイントを経由した場合でも符号の乱れがありません。しかしサーバ側でのバッファリングが必要になるためレイテンシが生じます。バッファーに何シンボル分溜まってから送出するか、規定シンボル数以下でも所定時間経過後に送出するか設定するパラメータがありますので適宜調節してください。

## Webサーバー機能
![サーバー画面](images/server.jpg)

サーバのIPアドレス(ポート80番)でHTTPサーバが動いています。この画面で統計情報の確認やパラメータの設定を行うことができます。

|統計情報　| 意味 |
|:---------|:--------------|
| Estimated Speed| 推定キーイング速度(WPM) |
| Estimated Dash Dot Ratio| 長点・短点の比 |
| Packet Error | パケットエラー数 |
|Packet Delay |パケットのディレイ(ms)|
| Queue Error | キュー溢れ回数 |
| Max queue length | キューに溜まったシンボルの最大個数 |
| Max mark duration | 長点の長さ(ms)|
| Space duration | 符号間の間隔(ms)

`Packet Error`は受信時にパケットのシーケンス番号が異なる（パケットがロスした）場合にカウントされます。`Packet Delay`とは符号パケットの到着時刻の間隔からキー入力時の符号間の間隔を引いたもので、この値が大きいほどネットワーク等の遅延が大きいことを表します。

|パラメータ     | 意味            |
|:-------------|:----------------|
| Queue Latency| 　キューされてから処理を始める時間|
| Symbol Wait  |　キューに保存するシンボル数 |

`Symbol Wait`とはキー出力までに入力キューに溜めるシンボル(長点・短点）の個数を指します。この値を長くするとネットワーク負荷によるタイミングのズレ（ジッタ）をより吸収することができます。

また入力キューに溜まっているシンボルが`Symbol Wait`以下であっても、シンボルがキューに入ってから以下の式で求められる`T`(ms)経過後はキー出力を行うようパラメータ`Queue Latency`を設定することができます。
```
 T = (長点の長さ(ms) + 符号間の間隔(ms)) * QueueLatency
```

これらのパラメータはネットワークの負荷状況や常用するキーイング速度によって最適な値が変わってきますので適宜修正するようにしてください。
