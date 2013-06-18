for i = 3000:3999
    fprintf('%d\n',i);
    im = imread(sprintf('/home/twangcat/Desktop/libviso2/I280stereo/left_%06d.png',i));
    imwrite(im, sprintf('le%04d.bmp',i-2999));
    im = imread(sprintf('/home/twangcat/Desktop/libviso2/I280stereo/right_%06d.png',i));
    imwrite(im, sprintf('ri%04d.bmp',i-2999));
end