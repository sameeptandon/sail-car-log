from os.path import join as pjoin
import subprocess
import pipes
from subprocess import check_call
from pipeline_config import CLUSTER_DSET_DIR, GPS_FILE, MAP_FILE, LDR_DIR,\
        PARAMS_FILE, CLUSTER_HOSTS
from fabric.colors import green


def exists_remote(host, path):
    return subprocess.call(
        ['ssh', host, 'test -e %s' % pipes.quote(path)]) == 0


def rsync_to_remote(local, remote, flags='-avs', exclude='.git', dry_run=False):
    cmd = 'rsync {flags} --exclude {exclude} {local} {remote}'.format(flags=flags,
            exclude=exclude, local=local, remote=remote)
    if dry_run:
        cmd += ' --dry-run'
    print cmd
    check_call(cmd, shell=True)


def scp_to_remote(local, remote):
    cmd = 'scp -r %s %s' % (local, remote)
    check_call(cmd, shell=True)


def tar_scp_to_remote(local, remote, dry_run=False):
    tar_file = local + '.tar'
    tar_cmd = 'tar -cvf %s %s' % (tar_file, local)
    print tar_cmd
    check_call(tar_cmd, shell=True)
    scp_to_remote(tar_file, remote)


if __name__ == '__main__':
    for host in CLUSTER_HOSTS:
        print green('syncing to %s' % host)
        rsync_to_remote(GPS_FILE, '%s:%s' % (host, CLUSTER_DSET_DIR))
        rsync_to_remote(MAP_FILE, '%s:%s' % (host, CLUSTER_DSET_DIR))
        rsync_to_remote(PARAMS_FILE, '%s:%s' % (host, CLUSTER_DSET_DIR))
        #tar_scp_to_remote(LDR_DIR, '%s:%s' % (host, CLUSTER_DSET_DIR))
        rsync_to_remote(LDR_DIR, '%s:%s' % (host, CLUSTER_DSET_DIR))
