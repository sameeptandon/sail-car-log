# python ClusterGenerateLaneLabels.py <hosts to use.txt> <folders to run>
import threading, Queue
import time, sys, os

RAW_DATA_LOC = '/scail/group/deeplearning/driving_data/raw_data/'
LANE_LAB_LOC = '/scail/group/deeplearning/driving_data/auto_lane_labels'
PER_WARP_LOC = '/scail/group/deeplearning/driving_data/perspective_transforms.pickle'
EXC_LIST_LOC = '~/scratch/sail-car-log/process/exclude_list'

queue = Queue.Queue()

def GenerateCommand(host, input_folder, output_folder): 
  #cmd = 'ssh sameep@%s "source ~/scratch/opencv/opencv/env_opencv_gorgon/bin/activate; cd ~/scratch/sail-car-log/process; python GenerateLaneLabels.py %s%s %s %s %s %s"' % (host, RAW_DATA_LOC, input_folder, output_folder, LANE_LAB_LOC, PER_WARP_LOC, EXC_LIST_LOC)
  cmd = 'ssh sameep@%s "echo hi"'
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
      time.sleep(0.1)
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
    host_list = map(lambda x: 'gorgon' + str(x), range(41,50))
  if sys.argv[2] == 'default':
    folder_list = ['7-16-sacramento', '7-18-101', '7-19-monterey', '7-24-101', '7-25-bay', '8-13-marin', '8-14-101', '8-15-tracy-gilroy']

  main(host_list, folder_list, sys.argv[3])

