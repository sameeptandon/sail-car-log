imgnames  = dir('*.bmp');

for i = 1:length(imgnames)
    
    im = imread( imgnames(i).name);
    im = imresize(im(401:401+531,:,:), [372,1344]);
    imwrite(im, sprintf('crop_calib%d.bmp',i));
end