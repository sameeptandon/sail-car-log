import numpy as np
import os
import glob
import shutil
import struct
import sys
from scipy.misc import toimage, imresize

def save_merged_file(file_name, imgs, labels, imgRows=60):
    """
    Saves the preprocessed images into a concatenated file. Saved file
    format [numimages, [factor, minval, img], [factor, min....]...]
    """
    assert(type(imgs) is list and type(imgs[0]) is np.ndarray)
    with open(file_name, 'wb') as out_file:
        # write the number of images
        out_file.write(struct.pack('i', len(imgs)))
        out_file.write(struct.pack('i', imgRows))

        for ind,img in enumerate(imgs):
            PIL_img = toimage(img)
            # record where we need to write image size
            nbytes_pos = out_file.tell()
            # skip ahead
            out_file.seek(4,1)

            curr_file_pos = out_file.tell()
            # save the image
            PIL_img.save(out_file, format='png')
            # jump back to write file size
            next_file_pos = out_file.tell()
            nbytes = next_file_pos - curr_file_pos
            out_file.seek(nbytes_pos)
            out_file.write(struct.pack('i', nbytes))
            out_file.seek(next_file_pos)
            # write labels
            for l in labels[ind]:
                out_file.write(struct.pack('f',l))

def read_merged_file(file_name, label_freq=1):
    """
    Given the name of a merged png file this will read each image and
    return a list of numpy arrays containing the images. This is the
    slow way to read a merged file. Use the load module if you're looking
    for a faster way to do this.
    """
    try:
        from PIL import Image, ContainerIO
    except:
        import Image
        import ContainerIO
        

    imgs = []
    labels = []
    with open(file_name, 'rb') as in_file:
        in_file.seek(0,2)
        end_pos = in_file.tell()
        in_file.seek(0)
        num_imgs = struct.unpack('i', in_file.read(4))[0]
        num_labels = struct.unpack('i', in_file.read(4))[0]

        it=0
        labels = []
        while in_file.tell() < end_pos:
            nbytes = struct.unpack('i', in_file.read(4))[0]
            curr_pos = in_file.tell()

            # load image into np array
            img = np.asarray(Image.open(ContainerIO.ContainerIO(in_file,
                                                                    curr_pos,
                                                                    nbytes)))

            in_file.seek(curr_pos + nbytes)

            img_labels = []
            for x in xrange(num_labels):
                lab = struct.unpack('i', in_file.read(4))[0]
                if x % label_freq == 0:
                    img_labels.append(lab)
            # add to list
            imgs.append(img)
            labels.append(img_labels)
            # seek past image
            it+=1

        # ensure we reached end of file
        curr_pos = in_file.tell()
        in_file.seek(0,2)
        assert(curr_pos == in_file.tell())

        # if these do not match throw warning
        if it != num_imgs:
            from warnings import warn
            warn('File %s reported %d images but only had %d' % (file_name,num_imgs,it))

    return imgs, labels
