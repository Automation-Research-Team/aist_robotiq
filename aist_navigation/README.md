# aist_navigation

移動台車を制御するためのパッケージです。

## 繰り返し精度の検証

1. Fetchを実験スペースの棚の前に移動させます。
   Fetchが棚を向いた時、Fetchの左側と背中側の端がそれぞれテープの内側に来るようにします。

2. **Odometryを初期化するため** 、Fetchを再起動します。

3. Fetchが起動したら、次のコマンドを入力してmove_baseを起動します。

   ```sh
   $ roslaunch aist_navigation move_base_mapless.launch
   ```

4. 3を実行したターミナルで、`odom received!` と表示されたら、別のターミナルを開き、次のコマンドを実行することでロボットの動作が開始します。

   ```sh
   $ rosrun aist_navigation check_reproducibility.py
   ```



### ファイルの出力先

`check_reproducibility.py` は、odometryの位置姿勢をx, y, z, roll, pitch, yawの形式で記録したCSVファイルを出力します。このCSVファイルは、`~/.ros/check_base_reproducibility` の下に、日時のフォルダを作成して、odom.csvというファイル名で保存されます。

単位は、x, y, zがm、roll, pick, yawがdeg.です。
