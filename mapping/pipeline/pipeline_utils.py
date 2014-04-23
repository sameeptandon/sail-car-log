import os
from subprocess import check_call

def file_num(file_path):
    return int(os.path.splitext(os.path.basename(file_path))[0])
