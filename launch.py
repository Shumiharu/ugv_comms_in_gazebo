import os
import sys
import time
import yaml
import subprocess

def usage():
  print('Usage: python3 launch.py <launch_config>')
  exit()

def is_gazebo_ready():
  ps_output = subprocess.check_output(['ps', 'aux'])
  ps_output_str = ps_output.decode('utf-8')
  print("waiting for running Gazebo")
  if 'ign gazebo gui' in ps_output_str:
      return True
  else:
      return False
  
if __name__ == '__main__':
  current_dirpath = os.getcwd()
  
  args = sys.argv
  if not len(args) == 2:
    usage()
    
  with open(f'./launch_config/{args[1]}.yaml', 'r') as f:
    config = yaml.safe_load(f)
  
  src = config['src']
  plugin = config['plugin']
  world = config['world']
  debug_level = config['debug_level']
  tx = config['tx']
  rx = config['rx']

  # built plugin
  try:
    cmd_build_plugin = f'cd ./src/{src}/build && make && sudo cp ./lib{plugin}.so /usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins/.'
    print(f'{current_dirpath}$ {cmd_build_plugin}')
    subprocess.run(cmd_build_plugin, shell=True, check=True)
  except subprocess.CalledProcessError as e:
    print(e)
    exit()
  
  print(f'system plugin "lib{plugin}.so" was built.')
  
  # build publisher
  try:
    cmd_build_publisher = 'cd ./src/comms/build && make'
    print(f'{current_dirpath}$ {cmd_build_publisher}')
    subprocess.run(cmd_build_publisher, shell=True, check=True)
  except subprocess.CalledProcessError as e:
    print(e)
    exit()
  
  print('publisher was built.')

  # converts xacro to urdf
  tx_dir = f'./models/{tx}s/xacro'
  tx_num = sum(os.path.isfile(os.path.join(tx_dir, name)) for name in os.listdir(tx_dir))
  # tx_num = 3
  for i in range(tx_num):
    _tx = f'{tx}_{i}'
    try:
      cmd_cvt_tx = f'xacro ./models/{tx}s/xacro/{_tx}.xacro  > ./models/{tx}s/urdf/{_tx}.urdf current_dirpath:={current_dirpath}'
      print(f'{current_dirpath}$ {cmd_cvt_tx}')
      proc_cvt_gs = subprocess.run(cmd_cvt_tx, shell=True)
    except subprocess.CalledProcessError as e:
      print(e)
      exit()

    print(_tx + '.xacro was converted to urdf')
  
  rx_dir = f'./models/{rx}s/xacro'
  rx_num = sum(os.path.isfile(os.path.join(rx_dir, name)) for name in os.listdir(rx_dir))
  for i in range(rx_num):
    try:
      cmd_cvt_rx = f'xacro ./models/{rx}s/xacro/{rx}_0.xacro  > ./models/{rx}s/urdf/{rx}_0.urdf current_dirpath:={current_dirpath}'
      print(f'{current_dirpath}$ {cmd_cvt_rx}')
      proc_cvt_rx = subprocess.run(cmd_cvt_rx, shell=True)
    except subprocess.CalledProcessError as e:
      print(e)
      exit()
  
  # start gazebo
  cmd_gz = f'exec ign gazebo -v{debug_level} ./worlds/{world}.sdf'
  print(f'{current_dirpath}$ {cmd_gz}')
  proc_gz = subprocess.Popen(cmd_gz, shell=True)

  # wait for running gazebo
  while not is_gazebo_ready():
    time.sleep(1)  

  print('Gazebo GUI is ready')

  # try setting tx and launching a comms publisher
  for i in range(tx_num):
    _tx = f'{tx}_{i}'
    try:
      cmd_set_tx = f'ign service -s /world/{world}/create \
                    --reqtype ignition.msgs.EntityFactory \
                    --reptype ignition.msgs.Boolean \
                    --timeout 5000 \
                    --req "sdf_filename: \'./models/{tx}s/urdf/{_tx}.urdf\', name: \'{_tx}\'"'
      print(f'{current_dirpath}$ {cmd_set_tx}')
      proc_set_gb = subprocess.run(cmd_set_tx,shell=True, check=True)
    except subprocess.CalledProcessError as e:
      print(e)
      exit()
  
  # try setting rx and launching a comms subscriber
  proc_tpcs = []
  for i in range(rx_num):
    _rx = f'{rx}_{i}'
    try:
      cmd_set_rx = f'ign service -s /world/{world}/create \
                    --reqtype ignition.msgs.EntityFactory \
                    --reptype ignition.msgs.Boolean \
                    --timeout 5000 \
                    --req "sdf_filename: \'./models/{rx}s/urdf/{_rx}.urdf\', name: \'{_rx}\'"'
      print(f'{current_dirpath}$ {cmd_set_rx}')
      proc_set_rx = subprocess.run(cmd_set_rx, shell=True, check=True)
    except subprocess.CalledProcessError as e:
      print(e)
      exit()

    # try subscribing published data (optional, when you record)
    cmd_tpc = f'exec ign topic -e -t {_rx}/rx'
    print(f'{current_dirpath}$ {cmd_tpc}')
    proc_tpc = subprocess.Popen(cmd_tpc, shell=True)
    proc_tpcs.append(proc_tpcs)
    print(f'{_rx} starts subscribe')
  
  proc_pubs = []
  for i in range(tx_num):
    _tx = f'{tx}_{i}'
    for j in range(rx_num):
      _rx = f'{rx}_{j}'
      cmd_pub = f'exec ./src/comms/build/publisher {_tx} {_rx}'
      print(cmd_pub)
      proc_pub = subprocess.Popen(cmd_pub, shell=True)
      proc_pubs.append(proc_pub)
      print(f'{_tx} - {_rx} start comms')
  
  # If Gazebo is killed (ex: Ctrl + c), child processes are killed
  if proc_gz.wait() == 0:
    # proc_tpc.kill()
    for proc_pub in proc_pubs:
      proc_pub.kill()

