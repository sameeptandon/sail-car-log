import numpy as np
import os
import glob
import shutil
import struct
import sys
from scipy.ndimage.filters import gaussian_filter, convolve
from scipy.misc import toimage, imresize

def sort_image_from_label_filenames(allImgs, suffix='.png'):
    """
    Takes a list of filenames that contain both images and their
    corresponding labels and separates them into two distinct lists.
    Assumes that all labels are named in the same way as their 
    corresponding images but with '_label' appended to the root of the
    name.
    """
    imgs = []
    labelImgs = []
    label_suffix = '_label' + suffix
    for name in allImgs:
        if name.endswith(label_suffix):
            labelImgs.append(name)
        else:
            imgs.append(name)

    imgs.sort()
    labelImgs.sort()
    if len(imgs) != len(labelImgs):
        from warnings import warn
        warn('Number of labels differs from number of images found')

    return imgs, labelImgs

def save_raw_merged_file(file_name, imgs, labelImgs):
    """
    Saves all the images and labels concatenated in a joint file. Format
    is [numpairs, [img_size, img, label_size, label], [img_size,...],...]
    """
    assert(type(imgs) is list and type(imgs[0]) is np.ndarray)
    assert(type(labelImgs) is list and type(labelImgs[0]) is np.ndarray)
    assert(len(imgs) == len(labelImgs))
    with open(file_name, 'wb') as out_file:
        # write the number of images
        out_file.write(struct.pack('i', len(imgs)))

        for ind,img in enumerate(imgs):
            PIL_img = toimage(img)
            PIL_label = toimage(labelImgs[ind])

            # record where to write img size
            img_size_pos = out_file.tell()
            out_file.seek(4, 1)

            # save img
            file_start_pos = out_file.tell()
            PIL_img.save(out_file, format='png')
            label_size_pos = out_file.tell()

            # write img size
            img_size = label_size_pos - file_start_pos
            out_file.seek(img_size_pos)
            out_file.write(struct.pack('i', img_size))
            out_file.seek(label_size_pos)

            # save label
            out_file.seek(4, 1)
            label_start_pos = out_file.tell()
            PIL_label.save(out_file, format='png')
            end_label_pos = out_file.tell()

            # write label size
            label_size = end_label_pos - label_start_pos
            out_file.seek(label_size_pos)
            out_file.write(struct.pack('i', label_size))
            out_file.seek(end_label_pos)

def read_raw_merged_file(file_name):
    """
    Given the name of a raw merged png file this will read each image and
    return a list of numpy arrays containing the images.
    """
    try:
        from PIL import Image, ContainerIO
    except:
        import Image
        import ContainerIO
    

    imgs = []
    labelImgs = []
    with open(file_name, 'rb') as in_file:
        in_file.seek(0,2)
        end_pos = in_file.tell()
        in_file.seek(0)
        num_imgs = struct.unpack('i', in_file.read(4))[0]

        it=0
        while in_file.tell() < end_pos:
            nbytes_file = struct.unpack('i', in_file.read(4))[0]
            
            curr_pos = in_file.tell()

            # load image into np array
            img = np.asarray(Image.open(ContainerIO.ContainerIO(in_file,
                                                                curr_pos,
                                                                nbytes_file)))
            # add to list
            imgs.append(img)
            # seek past image
            in_file.seek(curr_pos + nbytes_file)

            nbytes_label = struct.unpack('i', in_file.read(4))[0]
            label_pos = in_file.tell()

            #load label into np array
            label = np.asarray(Image.open(ContainerIO.ContainerIO(in_file,
                                                                 label_pos,
                                                                 nbytes_label)))
            # add to list
            labelImgs.append(label)

            in_file.seek(label_pos + nbytes_label)

            it+=1

        # ensure we reached end of file
        curr_pos = in_file.tell()
        in_file.seek(0,2)
        assert(curr_pos == in_file.tell())
        
        # if these do not match throw warning
        if it != num_imgs:
            from warnings import warn
            warn('File %s reported %d images but only had %d' % (file_name,num_imgs,it))

    return imgs, labelImgs

