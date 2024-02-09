import os
import time
import subprocess

def is_gazebo_ready():
  ps_output = subprocess.check_output(['ps', 'aux'])
  ps_output_str = ps_output.decode('utf-8')
  print("waiting for running Gazebo")
  if 'ign gazebo gui' in ps_output_str:
      return True
  else:
      return False
  
if __name__ == '__main__':

  # compile rf_comms_custom
  try:
    subprocess.run('cd ./src/rf_comms_custom/build && make && sudo cp ./libRFComms_custom.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/.', shell=True, check=True)
  except subprocess.CalledProcessError as e:
    print(e)
    exit()
  
  print('rf_comms_custom is compiled.')
  
  # compile publisher
  current_dirpath = os.getcwd()
  try:
    subprocess.run('cd ./src/comms/build && make', shell=True, check=True)
  except subprocess.CalledProcessError as e:
    print(e)
    exit()
  
  print('publisher is compiled.')

  ground_station_dir = './models/ground_station_antennas/xacro'
  ground_station_antenna_num = sum(os.path.isfile(os.path.join(ground_station_dir, name)) for name in os.listdir(ground_station_dir))
  # ground_station_antenna_num = 1
  
  # convert xacro to urdf (multicopter)
  try:
    proc_cvt_multicopter = subprocess.run(f'xacro ./models/multicopters/xacro/multicopter_0.xacro  > ./models/multicopters/urdf/multicopter_0.urdf current_dirpath:={current_dirpath}', shell=True)
  except subprocess.CalledProcessError as e:
    print(e)
    exit()
  
  # converts xacro to urdf (ground_station_antenna)
  for i in range(ground_station_antenna_num):
    ground_station_antenna = 'ground_station_antenna_' + str(i)
    try:
      proc_cvt_gs = subprocess.run(f'xacro ./models/ground_station_antennas/xacro/{ground_station_antenna}.xacro  > ./models/ground_station_antennas/urdf/{ground_station_antenna}.urdf current_dirpath:={current_dirpath}', shell=True)
    except subprocess.CalledProcessError as e:
      print(e)
      exit()

    print(ground_station_antenna + '.xacro was converted to urdf')
  
  # start gazebo
  debugging_level = 5
  proc_gz = subprocess.Popen('exec ign gazebo -v' + str(debugging_level) + ' ./worlds/rf_comms_custom2.sdf', shell=True)

  # wait for running gazebo
  while not is_gazebo_ready():
    time.sleep(1)  
  print('Gazebo GUI is ready')

  # try setting multicopter and launching a comms subscriber
  try:
    proc_set_multicopter = subprocess.run(f'ign service -s /world/rf_comms_custom2/create \
                  --reqtype ignition.msgs.EntityFactory \
                  --reptype ignition.msgs.Boolean \
                  --timeout 5000 \
                  --req "sdf_filename: \'./models/multicopters/urdf/multicopter_0.urdf\', name: \'multicopter_0\'"',
                  shell=True, check=True
                  )
  except subprocess.CalledProcessError as e:
    print(e)
  
  print('multicopter_0 is successed to set up')

  proc_tpc = subprocess.Popen('exec ign topic -e -t multicopter_0/rx', shell=True)
  
  # try setting ground_station_antennas and launching a comms publisher
  proc_pubs = []
  for i in range(ground_station_antenna_num):
    ground_station_antenna = 'ground_station_antenna_' + str(i)
    try:
      proc_set_gb = subprocess.run(f'ign service -s /world/rf_comms_custom2/create \
                    --reqtype ignition.msgs.EntityFactory \
                    --reptype ignition.msgs.Boolean \
                    --timeout 5000 \
                    --req "sdf_filename: \'./models/ground_station_antennas/urdf/{ground_station_antenna}.urdf\', name: \'{ground_station_antenna}\'"',
                    shell=True, check=True
                    )
    except subprocess.CalledProcessError as e:
      print(e)

    print(ground_station_antenna + ' is successed to set up')

    proc_pub = subprocess.Popen(f'exec ./src/comms/build/publisher {ground_station_antenna}', shell=True)
    proc_pubs.append(proc_pub)
  
  # If Gazebo is killed (ex: Ctrl + c), child processes are killed
  if proc_gz.wait() == 0:
    proc_tpc.kill()
    for proc_pub in proc_pubs:
      proc_pub.kill()

