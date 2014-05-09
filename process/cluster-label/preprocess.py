import numpy as np
import os
import glob
import struct
import sys
from scipy.ndimage.filters import gaussian_filter, convolve
from scipy.misc import toimage, imresize

def load_images(img_files):
    """
    Given a list of image file names, this loads and returns a list of images
    """
    from pylab import imread

    assert(type(img_files) is list and type(img_files[0]) is str)
    imgs = []
    for img_file in img_files:
        try:
            img = imread(img_file)
        except IOError:
            print "Could not load %s !!! skipping..." % img_file
            continue
        imgs.append(img)

    return imgs

def files_list(folder, suffix='.png'):
    """
    Returns a list of all the images within this directory.
    """
    return glob.glob(folder + '/*' + suffix)

def save_merged_file(file_name, imgs, labels=None):
    """
    Saves the preprocessed images into a concatenated file. Saved file
    format [numimages, [factor, minval, img], [factor, min....]...]
    """
    assert(type(imgs) is list and type(imgs[0]) is np.ndarray)
    with open(file_name, 'wb') as out_file:
        # write the number of images
        out_file.write(struct.pack('i', len(imgs)))

        for ind,img in enumerate(imgs):
            # write the length in bytes of file
            maxval = np.max(img)
            minval = np.min(img)
            # scale the image
            if (maxval-minval) == 0:
                img[:] = 0
            else:
                img = (img - minval)/(maxval-minval)
            # write the factor and minval
            PIL_img = toimage(img)
            # record where we need to write image size
            nbytes_pos = out_file.tell()
            # skip ahead
            out_file.seek(4,1)
            # write minval, maxval
            out_file.write(struct.pack('ff', minval, maxval))
            if labels is not None:
                out_file.write(struct.pack('f',labels[ind]))
            
            curr_file_pos = out_file.tell()                   
            # save the image
            PIL_img.save(out_file, format='png')
            # jump back to write file size
            next_file_pos = out_file.tell()
            nbytes = next_file_pos - curr_file_pos
            out_file.seek(nbytes_pos)
            out_file.write(struct.pack('i', nbytes))
            out_file.seek(next_file_pos)

def resize(imgs, size):
    if not isinstance(imgs,list):
        imgs=[imgs]
    rimgs = [imresize(img,size) for img in imgs]
    return rimgs

def save_pp_images(file_list, pp_imgs, labels=None):
    """
    saves each file individually into format [factor, minval, label, img]
    label is skipped if not provided
    """
    assert(len(file_list) == len(pp_imgs))
    for ind,img in enumerate(pp_imgs):
        with open(file_list[ind],'wb') as f:
            maxval = np.max(img)
            minval = np.min(img)
            img = (img-minval)/(maxval-minval)
            PIL_img = toimage(img)
            f.write(struct.pack('fff',minval,maxval,labels[ind]))
            PIL_img.save(f, format='png')

def load_pp_imgs(files):
    from warnings import warn
    warn("not yet tested, may be issues")
    if not isinstanc(files,list):
        files=[files]

    imgs = []
    labels = []
    for inf in files:
        with open(inf,'r') as f:
            minval,maxval,label=struct.unpack('fff', in_file.read(3*4))
            if minval > maxval:
                minval, maxval = maxval, minval
            factor = (maxval - minval)/255.0
            
            curr_pos = in_file.tell()
            in_file.seek(0,2)
            end_pos = in_file.tell()
            nbytes = end_pos - curr_pos
            img = np.asarray(Image.open(ContainerIO.ContainerIO(in_file,
                                                                curr_pos,
                                                                nbytes)))
            
            img = img * factor + minval
            labels.append(int(label+.5))

def read_merged_file(file_name):
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
    with open(file_name, 'rb') as in_file:
        in_file.seek(0,2)
        end_pos = in_file.tell()
        in_file.seek(0)
        num_imgs = struct.unpack('i', in_file.read(4))[0]

        it=0
        labels = []
        while in_file.tell() < end_pos:
            nbytes, minval, maxval = struct.unpack('iff', in_file.read(3*4))
            if minval > maxval:
                minval, maxval = maxval, minval
                
            factor = (maxval - minval)/255.0
            
            curr_pos = in_file.tell()

            # load image into np array
            try:
                img = np.asarray(Image.open(ContainerIO.ContainerIO(in_file,
                                                                    curr_pos,
                                                                    nbytes)))
            except IOError:
                in_file.seek(curr_pos)
                labels.append(int(struct.unpack('f',in_file.read(4))[0]+.5))
                curr_pos = in_file.tell()
                img = np.asarray(Image.open(ContainerIO.ContainerIO(in_file, curr_pos, nbytes)))
                
                                
            # re-whiten the image
            img = img * factor + minval
            # add to list
            imgs.append(img)
            # seek past image
            in_file.seek(curr_pos + nbytes)
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

def preprocess_images(raw_imgs):
    """
    Given a list of images, preprocess them and return a list of them. 
    """
    from scipy.io import loadmat

    assert(type(raw_imgs) is list and type(raw_imgs[0]) is np.ndarray)
    pp_imgs = []

    # load the matlab kernel
    script_dir = os.path.dirname(os.path.abspath(__file__))
    whiten_kernel = loadmat(os.path.join(script_dir,
                                          'whitening_kernels.mat'))['f']
    ten_per = len(raw_imgs)/10
    img_num = 0
    for raw_img in raw_imgs:
        # if ten_per > 0 and img_num % ten_per == 0:
        #     print('Preprocessing %d/%d' % (img_num, len(raw_imgs)))

        if raw_img.ndim == 2:
            raw_img = gray2rgb(raw_img)
        if raw_img.shape[2] == 4:
            raw_img = raw_img = raw_img[:,:,:3]
            
        # Gaussian filter the image
        filt_img = np.zeros_like(raw_img)
        for i in xrange(raw_img.shape[2]):
            filt_img[:,:,i] = gaussian_filter(raw_img[:,:,i], sigma=0.5)

        # Contrast normalzie the image
        lcn_img = lcn(filt_img, 16, 500)

        # whiten image
        white_img = whiten(lcn_img, whiten_kernel)

        # add to image list
        pp_imgs.append(white_img)

        img_num += 1
        
    return pp_imgs

def gray2rgb(im):
    w, h = im.shape
    ret = np.empty((w, h, 3), dtype=np.uint8)
    ret[:, :, 2] =  ret[:, :, 1] =  ret[:, :, 0] =  im
    return ret

def whiten(img, whiten_kernel):
    filt_img = np.zeros_like(img)
    for i in xrange(img.shape[2]):
        for j in xrange(img.shape[2]):
            filt_img[:,:,i] = filt_img[:,:,i] + convolve(img[:,:,j], whiten_kernel[i,:,:,j])
            
    return filt_img

def process_image_file(img_file):
    """
    Given an image file, this will return the processed image
    """
    return preprocess_images( load_images([img_file]) )[0]

def lcn(img, N, EPS):
    """
    Local Contrast Normalization
    """
    # and another gaussian filter...
    filt_img = np.zeros_like(img)
    for i in xrange(img.shape[2]):
        filt_img[:,:,i] = gaussian_filter(img[:,:,i], sigma=(N/4.0))

    # mean zero the image
    mz_img = (img.transpose() - np.mean(filt_img,2).transpose()).transpose()

    # get the variance
    vr = np.mean(mz_img**2,2)
    vr = gaussian_filter(vr, sigma=(N/4.0))
    vr = np.sqrt(np.maximum(vr, 0.01) + EPS)

    # get the normalized image
    nrm_img = (mz_img.transpose() / vr.transpose()).transpose()
    
    return nrm_img

def pltable(img):
    """
    Make the image plotable by putting values between [0-1]
    """
    min_val = np.min(img)
    max_val = np.max(img)

    return (img - min_val)/(max_val-min_val)
    
    

