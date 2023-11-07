# ugv_comms_in_gazebo

customaize rf_comms.cc -> rf_comms_custom.cc

cd ugv_comms_in_gazebo/model/rf_comms_custom
mkdir build 
cd build 
cmake ..
make
sudo cp ./libRFComms_custom.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/.

boot gazebo fortress
ign gazebo rf_comms_custom.sdf
if you show ignwrn/igndbg in rf_comms/rf_comms_custom
ign gazebo -v<NUMBER> rf_comms_custom.sdf

publish message from base_station to ugv
cd ugv_comms_in_gazebo/model/comms
mkdir build
cd build
cmake ..
make
./publisher ugv

listen message from base_station to ugv
ign topic -e -t ugv/rx