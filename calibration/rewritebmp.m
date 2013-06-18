imgs = dir('*.png')

for i = 1:length(imgs)
    im = imread(imgs(i).name);
    imwrite(im, [imgs(i).name(1:end-4), '.bmp']);
end