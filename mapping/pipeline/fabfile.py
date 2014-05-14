import os
from os.path import join as pjoin
from fabric.api import env, run, task
from pipeline_config import CLUSTER_DSET_DIR, FABRIC_PASS_FILE, CLUSTER_HOSTS

# PARAM
env.hosts = CLUSTER_HOSTS
env.password = open(FABRIC_PASS_FILE, 'r').read().strip()
env.colorize_errors = True

@task
def mkdirs():
    run('mkdir -p %s' % CLUSTER_DSET_DIR)


if __name__ == '__main__':
    pass
