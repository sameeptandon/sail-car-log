import os
from fabric.colors import green
from pipeline_config import CLUSTER_DSET_DIR, CLUSTER_HOSTS, FILTERED_CLOUDS_DIR,\
        MERGED_CLOUDS_DIR


from subprocess import check_call

def scp_to_local(remote, local):
    cmd = 'scp -r %s %s' % (remote, local)
    print cmd
    check_call(cmd, shell=True)

if __name__ == '__main__':
    for host in CLUSTER_HOSTS[0:1]:
        print green('syncing from %s' % host)
        #scp_to_local('%s:%s' % (host, os.path.join(CLUSTER_DSET_DIR, 'filtered_clouds')), FILTERED_CLOUDS_DIR + '_remote')
        scp_to_local('%s:%s' % (host, os.path.join(CLUSTER_DSET_DIR, 'merged_clouds')), MERGED_CLOUDS_DIR + '_remote')
