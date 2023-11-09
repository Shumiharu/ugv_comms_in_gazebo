# ugv_comms_in_gazebo

## rf_comms.cc -> rf_comms_custom.cc にカスタマイズする
gz-sim/src/systems/rf_commsのフォルダごとコピーしてrf_comms.ccからrf_comms_custom.ccとしてカスタマイズします（他のプラグインを書き換える際もそのほうが簡単です）
書き換えたrf_comms_custom.ccをGazeboが読み込めるようにするには
```
cd ugv_comms_in_gazebo/model/rf_comms_custom
mkdir build 
cd build 
cmake ..
make
sudo cp ./libRFComms_custom.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/.
```

さらにプラグイン参照先も変更してください。

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
```
ign gazebo rf_comms_custom.sdf
```

rf_comms_custom.cc のignwrn/igndbgなど表示させたい場合は
```
ign gazebo -v<$NUMBER> rf_comms_custom.sdf
```

## 送信
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
