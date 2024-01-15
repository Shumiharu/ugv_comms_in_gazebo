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
  try:
    subprocess.run('cd ./src/comms/build && make', shell=True, check=True)
  except subprocess.CalledProcessError as e:
    print(e)
    exit()
  
  print('publisher is compiled.')

  base_station_dir = './models/base_stations/xacro'
  base_station_num = sum(os.path.isfile(os.path.join(base_station_dir, name)) for name in os.listdir(base_station_dir))
  # base_station_num = 1
  
  # convert xacro to urdf
  for i in range(base_station_num):
    base_station = 'base_station_' + str(i)

    try:
      proc_cvt = subprocess.run(f'xacro ./models/base_stations/xacro/{base_station}.xacro  > ./models/base_stations/urdf/{base_station}.urdf', shell=True)
    except subprocess.CalledProcessError as e:
      print(e)
      exit()

    print(base_station + '.xacro was converted to urdf')
  
  # start world
  debugging_level = 0
  proc_gz = subprocess.Popen('exec ign gazebo -v' + str(debugging_level) + ' ./worlds/rf_comms_custom.sdf', shell=True)

  # wait for running gazebo
  while not is_gazebo_ready():
    time.sleep(1)  
  print('Gazebo GUI is ready')

  # try launching a comms subscriber
  proc_tpc = subprocess.Popen('exec ign topic -e -t ugv/rx', shell=True)

  # try setting base_stations and launching a comms publisher
  proc_pubs = []
  for i in range(base_station_num):
    base_station = 'base_station_' + str(i)
    try:
      proc_set = subprocess.run(f'ign service -s /world/rf_comms/create \
                    --reqtype ignition.msgs.EntityFactory \
                    --reptype ignition.msgs.Boolean \
                    --timeout 5000 \
                    --req "sdf_filename: \'./models/base_stations/urdf/{base_station}.urdf\', name: \'{base_station}\'"',
                    shell=True, check=True
                    )
    except subprocess.CalledProcessError as e:
      print(e)

    print(base_station + ' is successed to set up')

    proc_pub = subprocess.Popen(f'exec ./src/comms/build/publisher {base_station}', shell=True)
    proc_pubs.append(proc_pub)
  
  # If Gazebo is killed (ex: Ctrl + c), child processes are killed
  if proc_gz.wait() == 0:
    proc_tpc.kill()
    for proc_pub in proc_pubs:
      proc_pub.kill()





