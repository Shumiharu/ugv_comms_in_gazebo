# ugv_comms_in_gazebo

## rf_comms.cc -> rf_comms_custom.cc にカスタマイズする
rf_commsの

```
cd ugv_comms_in_gazebo/model/rf_comms_custom
mkdir build 
cd build 
cmake ..
make
sudo cp ./libRFComms_custom.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/.
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
