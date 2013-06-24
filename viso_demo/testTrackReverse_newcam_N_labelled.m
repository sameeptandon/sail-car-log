% this file contains the cell array Tr_total.
% Tr_total{i} gives the 4x4 homogenous transformation matrix of the i-th
% frame wrt 1st frame. So Tr_total{1} is identity.
load Tr_280N_5_16_2013_f_right.mat;

% this file contains xall and yall, which are the human labelled 
% lane marking points (2 per image).
load 280N_f_right_labels.mat;


labelstep = 120; % skip this many frames in between frames
totalImg = length(Tr_total);

% camera intrinsics (simplified)
f     = 2271.3;
cu    = 622.0338;
cv    = 419.4885;

% camera extrinsics (these values should be ok. If not, we can always measure again)
height = 1.106; % height above ground in meters 
pitch  = 0.036; % elevation angle in radians


KK = [f 0 cu; 0 f cv; 0 0 1]; % homogenous camera matrix
Tc = [1 0 0 0; 0 cos(pitch) -sin(pitch) -height; 0 sin(pitch) cos(pitch) 0; 0 0 0 1]; % rotation matrix
p2 = 0.00;
Tc2 = [1 0 0 0; 0 cos(p2) -sin(p2) 0 ; 0 sin(p2) cos(p2) 0; 0 0 0 1];

img_dir = 'imgs/'; % replace this with your own directory holding the data
startImg = 1000;

imprefix = '280N_f_right';

startlabelImg = 1121; 
allLeftLane = [];
allRightLane = [];
totalImg = 10601;
Cnt = 1;
 for imgnum = startlabelImg:totalImg
    
        %system(['cp ' [img_dir imprefix num2str(imgnum,'%05d') '.png '] '/afs/cs/group/photo_ocr/scr/Data_for_DH/']);
        I = imread([img_dir imprefix num2str(imgnum,'%05d') '.png']);
        
           
        leftLane = [];
        rightLane = [];
        % plot points on current image
        for i = 0:1:labelstep
            
            % 3d poisition of the 2 labeled points labeled in the ith
            % future frame wrt their own camera frame 
            x = xall(1, imgnum-startlabelImg+1+i);
            y = yall(1, imgnum-startlabelImg+1+i);
            Z = ((y-cv)*sin(pitch)*height+f*cos(pitch)*height)/(cos(pitch)*(y-cv)-f*sin(pitch));
            X = (cos(pitch)*Z-sin(pitch)*height)*(x-cu)/f;
            Y=0;

            x1 = xall(2, imgnum-startlabelImg+1+i);
            y1 = yall(2, imgnum-startlabelImg+1+i);            
            Z1 = ((y1-cv)*sin(pitch)*height+f*cos(pitch)*height)/(cos(pitch)*(y1-cv)-f*sin(pitch));
            X1 = (cos(pitch)*Z1-sin(pitch)*height)*(x1-cu)/f;
            Y1=0;
            
            % position of those 2 points in world frame
            Pos = (Tr_total{imgnum+i-startImg+1})*(Tc\[X;Y;Z;1]);
            Pos1 = (Tr_total{imgnum+i-startImg+1})*(Tc\[X1;Y1;Z1;1]);
            
            % map to 3-d positions wrt the current camera frame, then 2d
            % pixel positions in the image.
            Pos2 = Tc2\((Tr_total{imgnum-startImg+1})\Pos);
            leftLane = [leftLane, Pos2];
            pos2 = round(KK*Pos2(1:3)/Pos2(3));
            
            I(pos2(2)-3:pos2(2)+3, pos2(1)-3:pos2(1)+3, 1) = 255;
            I(pos2(2)-3:pos2(2)+3, pos2(1)-3:pos2(1)+3, 2) = 0;
            I(pos2(2)-3:pos2(2)+3, pos2(1)-3:pos2(1)+3, 3) = 0;
            
            
            
            Pos2 = Tc2\((Tr_total{imgnum-startImg+1})\Pos1);
            rightLane = [rightLane, Pos2];  
            pos2 = round(KK*Pos2(1:3)/Pos2(3));
            
            
            I(pos2(2)-3:pos2(2)+3, pos2(1)-3:pos2(1)+3, 1) = 0;
            I(pos2(2)-3:pos2(2)+3, pos2(1)-3:pos2(1)+3, 2) = 255;
            I(pos2(2)-3:pos2(2)+3, pos2(1)-3:pos2(1)+3, 3) = 0;
            
        end
        % visualize lanes:
        I= imresize(I,0.5);
        
        imshow(I);
        drawnow;
        allLeftLane{Cnt}=leftLane;
        allRightLane{Cnt}=rightLane;
        Cnt = Cnt+1;
 end
% save results if you wish:
% save data_for_david2.mat f cu cv height pitch KK Tc allLeftLane allRightLane


