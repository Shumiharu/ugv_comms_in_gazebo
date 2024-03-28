# ugv_comms_in_gazebo

## カスタムプラグインの作成方法

## requirement
Ubuntu: 22.04
ROS2: Humble
Ignition Gazebo: Fortress

参考: https://qiita.com/porizou1/items/5dd915402e2990e4d95f

## Usage 
```
python3 ./launch.py <launch_config>
```
launch_configは./launch_configにあるyamlファイルを参照します。
yamlの構成は以下の通りです。
```
src: ./srcにあるシステムプラグインのディレクトリ名を入力する。通常はrf_comms_custom
plugin: システムプラグインの名前を入力する。通常はRFComms_custom
world: シミュレーション実行ファイル（.sdf）の名前を入力する。
debug_level: Gazeboのメッセージの出力レベルを入力。おおよそ0か5で良い。
tx: ./modelsにある送信側モデル名を入力する。（単数形）ground_station_antennaなど
rx: ./modelsにある受信側モデル名を入力する。（単数形）
```
また、launch.pyは以下を同時に処理します（実行したコマンドがターミナルに出力されます。したがってエラーが発生した場合はまずコマンドが正しく入力されているか確認してください）。各処理の詳細については後述します。
1. ./src/comms/publisher.cc のビルド
2. ./src/rf_comms_custom/rf_comms_custom.cc のビルド & ビルドされた実行ファイル libRFComms_custom.so を該当ディレクトリにコピー
3. ./modelsにある地上局(Tx)とモビリティ(Rx)のモデルをXML macro（xacro）からunified robot description format（urdf）に変換
4. Gazebo GUIの起動
5. Gazebo GUIに3.のモデルを生成
6. publish（送信）の開始
7. subscribe（受信）の開始
8. launch.pyのプログラムの強制終了（ctrl + c）待機 & 終了後に子プロセスを強制終了

Gazebo GUI起動後、以下の操作を行うと、モビリティを操作できます。
1. ウィンドウ右上部の「︙」をクリック
2. その中から「Key Publisher」をクリック
3. ウィンドウ左下部の「再生（▶）」をクリック
4. キーボードの「↑」などを押下するとモビリティが動きます。詳しくはシミュレーション実行ファイル（.sdf）を参照してください。

## 1. ./src/comms/publisher.cc のビルド
```
cd ./src/comms/build
mkdir build
cd build
cmake ..
make
```
なお、launch.pyではbuild出力ディレクトリ（./src/comms/build）は作成されている前提で実行している点に注意してください。
オリジナルはgz-sim/examples/standalone/commsにあります。

## 2. ./src/rf_comms_custom/rf_comms_custom.cc のビルド & ビルドされた実行ファイル libRFComms_custom.so を該当ディレクトリにコピー
rf_comms_customはGazeboのシステムプラグインとなるので、Gazeboが起動時にReadするために該当のディレクトリにコピーする必要があります。
```
cd ./src/rf_comms_custom/build
mkdir build
cd build
cmake ..
make
sudo cp ./libRFComms_custom.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/.
```
なお、launch.pyではbuild出力ディレクトリ（./src/rf_comms_custom/build）は作成されている前提で実行している点に注意してください。
また、sudoのため、管理者のパスワードを求められます。

## 3. ./modelsにある地上局とモビリティのモデルをXML macro（xacro）からunified robot description format（urdf）に変換
Gazeboで用いられるSDFormat（SDF）でも地上局やモビリティなどのモデルを記述できますが、冗長になってしまうためxacroで記述し、urdfに変換することでこれを解決しました。
```
xacro ./models/${MODEL_DIR}/xacro/${MODEL_NAME}.xacro  > ./models/${MODEL_DIR}/urdf/${MODEL_NAME}.urdf
```
なお、./modelsの構成は
```
models
¦-${MODEL_NAME}s
¦ ¦-xacro
¦ ¦ ¦-${MODEL_NAME}_0.xacro
¦ ¦ ¦-         .
¦ ¦ ¦-         .
¦ ¦ ¦-         .
¦ ¦-urdf
¦ ¦ ¦-${MODEL_NAME}_0.urdf <- 生成ファイル
¦ ¦ ¦-         .
¦ ¦ ¦-         .
¦ ¦ ¦-         .
¦-xacro_base
  ¦-${MODEL_NAME}.xacro
```
のようになっており、モデルのベースは./models/xacro_base/${MODEL_NAME}.xacroに記述されており、各モデルの座標といった固有のパラメータは./models/${MODEL_NAME}s/xacro/${MODEL_NAME}_0.xacroに記載されています。上記コマンドにより、./models/${MODEL_NAME}s/xacro/${MODEL_NAME}_0.urdfが生成されます。

## 4. Gazebo GUIの起動
RF_comms_custom.ccなどのシステムプラグインのプログラムにあるigndbgやignwrnなどは、オプション -v でデバッグレベルを設定することで表示/非表示にできます。
```
ign gazebo -v${DEBUGGING_LEVEL} ./worlds/${WORLD_NAME}.sdf
```
なお、${WORLD_NAME}は${WORLD_NAME}.sdfのworldタグのnameと同一になるようにしてください。
また、./world/rf_comms_custom.sdfについて、プラグインのタグの引数にファイルパスを入れるところがありますが、必要に応じて変更してください。（相対パスも設定可能です）

```
<plugin
  filename="RFComms_custom"
  name="ignition::gazebo::systems::RFComms_custom">
  <file_config>
    <comms_analysis>./comms_analysis/</comms_analysis> <!-- ここ -->
  </file_config>
  <range_config>
    <!-- 中略 -->
  </range_config>
  <radio_config>
    <!-- 中略 -->
    <antenna_gains_dir_path>./antenna_gains/</antenna_gains_dir_path>　<!-- ここ -->
  </radio_config>
</plugin>
```

## 5. Gazebo GUIに3.のモデルを生成
SDFに記述したモデルはGazebo起動と同時に生成されますが、urdfの場合はGazebo起動後に別途生成する必要があります。
```
ign service -s ./world/${WORLD_NAME}/create \
                  --reqtype ignition.msgs.EntityFactory \
                  --reptype ignition.msgs.Boolean \
                  --timeout 5000 \
                  --req 'sdf_filename: "./models/${MODEL_NAME}s/urdf/${MODEL_NAME}_0.urdf", name: "${MODEL_NAME}_0"'

// example
ign service -s /world/rf_comms_custom2/create \
                  --reqtype ignition.msgs.EntityFactory  \
                  --reptype ignition.msgs.Boolean \
                  --timeout 5000 \
                  --req 'sdf_filename: "./models/multicopters/urdf/multicopter_0.urdf", name: "multicopter_0"'

ign service -s /world/rf_comms_custom2/create \
                  --reqtype ignition.msgs.EntityFactory  \
                  --reptype ignition.msgs.Boolean \
                  --timeout 5000 \
                  --req 'sdf_filename: "./models/ground_station_antennas/urdf/ground_station_antenna_0.urdf", name: "ground_station_antenna_0"'
```
参考: https://gazebosim.org/docs/fortress/spawn_urdf

## 6. publish（送信）の開始
```
./src/comms/build/publisher ${GROUND_STATION_ANTENNA_MODEL_NAME}

// example 
./src/comms/build/publisher ground_station_antenna_0
```
## 7. subscribe（受信したメッセージの表示）の開始
```
ign topic -e -t ${MOBILITY_MODEL_NAME}/rx

// example
ign topic -e -t multicopter_0/rx
```

## 8. launch.pyのプログラムの強制終了（ctrl + c）待機 & 終了後に子プロセスを強制終了
Pythonのサブプロセスは親プロセス（launch.py）を強制終了させても子プロセス（6.や7.のコマンド）は強制終了せず、PCの処理能力が低下するので注意。




## urdfが正しく記述されているか確認する
check_urdfを実行するにはiburdfdom-toolsをインストールしてください。
```
sudo apt install -y liburdfdom-tools
check_urdf ./models/ground_stations/urdf/ground_station_1.urdf
```

## SDFormatに変換したurdfの確認
```
ign sdf -p ${URDF_FILE} 
```

## システムプラグインのカスタマイズ方法
デフォルトのシステムプラグインをカスタマイズする方法を以下に示しています。
rf_comms.cc -> rf_comms_custom.cc
```
// Before
IGNITION_ADD_PLUGIN(RFComms,
                    ignition::gazebo::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RFComms,
                          "ignition::gazebo::systems::RFComms")
```

```
// After
IGNITION_ADD_PLUGIN(RFComms_custom,
                    ignition::gazebo::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RFComms_custom,
                          "ignition::gazebo::systems::RFComms_custom")
```
なお、前述の通り、rf_comms_custom.cc がビルドされた実行ファイル libRFComms_custom.so は /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/. にコピーする必要があります。

rf_comms.sdf -> rf_comms_custom.sdf
```
<!-- Before -->
<plugin
  filename="ignition-gazebo-rf-comms-system"
  name="ignition::gazebo::systems::RFComms">
  <!-- 中略 -->
</plugin>
```

```
<!-- After -->
<plugin
  filename="RFComms_custom"
  name="ignition::gazebo::systems::RFComms_custom">
  <!-- 中略 -->
</plugin>
```
参考: 
1. https://gazebosim.org/api/gazebo/4.2/createsystemplugins.html 
2. https://nullpo24.hatenablog.com/entry/2020/11/14/155631