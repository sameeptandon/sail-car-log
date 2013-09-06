# python ClusterGenerateLaneLabels.py <hosts to use.txt> <folders to run>
import threading, Queue
import time, sys, os

RAW_DATA_LOC = '/scail/group/deeplearning/driving_data/raw_data/'
LANE_LAB_LOC = '/scail/group/deeplearning/driving_data/auto_lane_labels'
PER_WARP_LOC = '/scail/group/deeplearning/driving_data/perspective_transforms.pickle'
EXC_LIST_LOC = '~/scratch/sail-car-log/process/exclude_list'

killCmd = False

queue = Queue.Queue()

def getDirs(rootdir):
  dirs = [ ] 
  files = os.listdir(rootdir)
  for f in files:
    if os.path.isdir(rootdir + '/' +  f):
      dirs.append(getDirs(rootdir + '/' + f))  
  
  if len(dirs) == 0:
    dirs = rootdir
  return dirs

def flatten(S):
  if S == []:
    return S
  if isinstance(S[0], list):
    return flatten(S[0]) + flatten(S[1:])
  return S[:1] + flatten(S[1:])

def GenerateCommand(host, input_folder, output_folder): 
  cmd = 'ssh sameep@%s "source ~/scratch/opencv/opencv/env_opencv_gorgon/bin/activate; cd ~/scratch/sail-car-log/process; python GenerateAllLabels.py %s %s %s %s %s"' % (host, input_folder, output_folder, LANE_LAB_LOC, PER_WARP_LOC, EXC_LIST_LOC)

  if killCmd == True:
    cmd = "ssh sameep@%s 'killall -u sameep python'" % (host)
  return cmd


class GorgonThread(threading.Thread):
  def __init__(self, name, queue, output_folder):
    threading.Thread.__init__(self)
    self.name = name
    self.queue = queue
    self.output_folder = output_folder

  def run(self):
    while not self.queue.empty(): 
      folder = self.queue.get()
      job = GenerateCommand(self.name, folder, self.output_folder)
      print self.name, ':', job
      os.system(job)
      time.sleep(0.5)
      self.queue.task_done() 

def main(host_list, folder_list, output_folder):
  for folder in folder_list:  
    queue.put(folder)

  for host in host_list:
    t = GorgonThread(name=host, queue=queue, output_folder=output_folder)
    t.start();

  queue.join()

if __name__ == '__main__':
  if sys.argv[1] == 'default':
    host_list = map(lambda x: 'gorgon' + str(x), range(41,60))
  
  if sys.argv[2] == 'default':
    folder_list = ['7-16-sacramento', '7-18-101', '7-19-monterey', '7-24-101', '7-25-bay', '8-13-marin', '8-14-101', '8-15-tracy-gilroy']
    folder_list = map(lambda x: "%s%s" % (RAW_DATA_LOC, x), folder_list)
  else:
    folder_list = flatten(getDirs(sys.argv[2]))

  if '--kill' in sys.argv:
    killCmd = True
    folder_list = host_list
  
  main(host_list, folder_list, sys.argv[3])

