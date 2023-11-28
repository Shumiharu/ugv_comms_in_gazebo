# ugv_comms_in_gazebo

## rf_comms.cc -> rf_comms_custom.cc にカスタマイズする
gz-sim/src/systems/rf_commsのフォルダをmodel/にコピーし、rf_commsからrf_comms_customとしてカスタマイズしています（おおよそRFComms->RFComms_customに名称変更すれば使えるはずです）。例えば、

変更前 rf_comms.cc line 475
```
IGNITION_ADD_PLUGIN(RFComms,
                    ignition::gazebo::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RFComms,
                          "ignition::gazebo::systems::RFComms")
```

変更後 rf_comms_custom.cc
```
IGNITION_ADD_PLUGIN(RFComms_custom,
                    ignition::gazebo::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RFComms_custom,
                          "ignition::gazebo::systems::RFComms_custom")
```

カスタマイズしたrf_comms_custom.ccをGazeboが読み込めるようにするには
```
cd ugv_comms_in_gazebo/model/rf_comms_custom
mkdir build 
cd build 
cmake ..
make
sudo cp ./libRFComms_custom.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/.
```
のようにビルドして生成されたlibRFComms_custom.soをGazeboがプラグインを参照するパス/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/にコピーする必要があります。

さらにプラグイン参照先も変更してください

変更前 rf_comms.sdf
```
<plugin
  filename="ignition-gazebo-rf-comms-system"
  name="ignition::gazebo::systems::RFComms">
  <range_config>
    <max_range>500000.0</max_range>
    <fading_exponent>2.6</fading_exponent>
    <l0>40</l0>
    <sigma>10.0</sigma>
  </range_config>
  <radio_config>
    <capacity>1000000</capacity>
    <tx_power>20</tx_power>
    <noise_floor>-90</noise_floor>
    <modulation>QPSK</modulation>
  </radio_config>
</plugin>
```

変更後 rf_comms_custom.sdf
```
<plugin
  filename="RFComms_custom"
  name="ignition::gazebo::systems::RFComms_custom">
  <range_config>
    <max_range>500.0</max_range>
    <fading_exponent>2.0</fading_exponent>
    <l0>68</l0>
    <sigma>3.0</sigma>
  </range_config>
  <radio_config>
    <center_frequency>600000000</center_frequency>
    <capacity>600000000</capacity>
    <tx_power>22.5</tx_power>
    <noise_floor>-90</noise_floor>
    <modulation>QPSK</modulation>
  </radio_config>
</plugin>
```

## Gazebo Fortressを起動する

rf_comms pluginのパラメータの読み込み先ファイルパス（絶対パス）をそれぞれ変更してください。相対パスで読み込める方法あれば連絡ください

comms_analysisタグ
/home/haruki/Desktop/ugv_comms_in_gazebo/comms_analysis/ -> （任意のフォルダ）/ugv_comms_in_gazebo/comms_analysis/

antenna_gains_dir_pathタグ
/home/haruki/Desktop/ugv_comms_in_gazebo/antenna_gains/　-> （任意のフォルダ）/ugv_comms_in_gazebo/antenna_gains/

```
ign gazebo rf_comms_custom.sdf
```

rf_comms_custom.cc のignwrn/igndbgなど表示させたい場合は
```
ign gazebo -v<$NUMBER> rf_comms_custom.sdf
```

## 送信
gz-sim/examples/standalone/commsをフォルダごとコピーしてmodelフォルダにコピーしています
```
cd ugv_comms_in_gazebo/model/comms
mkdir build
cd build
cmake ..
make
./publisher ugv
```
## 受信
```
ign topic -e -t ugv/rx
```

## 上記のプログラムを一斉に立ち上げる
```
ign launch comms.ign worldName:=rf_comms_custom
```
