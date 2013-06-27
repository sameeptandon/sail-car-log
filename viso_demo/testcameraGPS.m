close all;
clear all;
clc;

img_dir = '../data/280S_a1_img/'; % replace this with your own directory holding the data
imprefix = '280S_a1_';
data=csvread('280S_a_gps.out.csv');
% img_dir = '../data/HW1S_b1_img/';
% imprefix = 'HW1S_b1_';
% data=csvread('HW1S_b_gps.out.csv');

start_img = 18000; % 18000 flat 
num_images_fwd = 300; 
num_vid_frames = 500; 
% helpers
time_idx = 1;
pos_idx = 2:4;
vel_idx = 5:7;
angle_idx = 8:10; 

figure;


all_frames = cell(num_vid_frames,1);

for start_frame = start_img:start_img + num_vid_frames

    % get orientation of camera in world coordinate frame
    yaw_start = deg2rad(data(start_frame,10));
    pitch_start = -deg2rad(data(start_frame,9));
    roll_start = deg2rad(data(start_frame,8));
    height_start = data(start_frame,4);

    %pitch_start = asin(data(start_img,7)/norm(data(start_img,vel_idx)));

    R_to_i_from_w = angle2dcm(yaw_start,pitch_start,roll_start,'ZXY')'; 
    R_to_c_from_i = ...
             [-1 0 0;
              0 0 -1;
              0 -1 0]; 
    R_to_c_from_i = angle2dcm(deg2rad(1.0),0,0,'XYZ') * R_to_c_from_i;
    %pitch_start = 0;
    %roll_start 

    C = [0.0, 0.0, 0.0]'; % camera position in world coordinates
    %R = eye(3); % camera orientation
    %R = R_i_w'; 
    R = R_to_i_from_w;
    T = -R*C; 



%     cam.R = R; 
%     cam.T = T; 
%     cam.E = [cam.R cam.T; 0 0 0 1];
    cam.f = 2271.3;
    cam.cu = 622.0338;
    cam.cv = 419.4885; 
    % cam.f = 1280;
    % cam.cu = 640;
    % cam.cv = 640; 
    cam.KK = [cam.f 0 cam.cu;
              0 cam.f cam.cv; 
              0 0     1 ];

%     figure;
%     hold on;
%     plotcamera(cam.R,cam.T*1000); % 1000 for scaling mm to meters 
    %plotcamera(eye(3),zeros(3,1))

    % integrate velocity
    pts = zeros(num_images_fwd,3);
%     for t = 2:num_images_fwd
%         dt = data(start_img +t,time_idx) - data(start_img+t-1,time_idx);
%         pts(t,:) = pts(t-1,:) + dt*data(start_img+t-1,vel_idx); 
%     end
%     pts(:,[1,2]) = pts(:,[2,1]);
%     pts(:,3) = data(start_img+1:start_img+num_images_fwd,4)-height_start-1.1;
    for t = 1:num_images_fwd-1
       pts(t+1,:) = dllh2denu(data(start_frame,pos_idx), data(start_frame+t,pos_idx)); 
    end
    pts(:,3) = pts(:,3)-1.1; 


%     size(pts)
%     %vis_pts = (angle2dcm(pi/2,0,0,'XYZ')*pts')';
%     vis_pts = pts;
% 
%     scatter3(vis_pts(:,1), vis_pts(:,2), vis_pts(:,3));
% 
%     xlim([-0.5 0.5])
%     ylim([-0.5 0.5])
%     zlim([-0.5 0.5])
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')

    cam_pix = zeros(num_images_fwd,2);

    %figure;
    I = imread([img_dir imprefix num2str(start_frame,'%0d') '.jpg']);
    for idx = 2:size(pts,1)
        idx;
        %pt = [pts(idx,1:3), 1]';
        %pos_wrt_imu = cam.E * pt;
        world_coordinates = pts(idx,1:3)';
        pos_wrt_imu = R_to_i_from_w * pts(idx,1:3)';
        %pos_wrt_imu(3) = data(start_img+idx,4)-height_start
        %pos_wrt_imu(3) = -1;
    %     T_pos_wrt_imu = [eye(3), pos_wrt_imu(1:3) + [0.25 -2.13 0]'; 0 0 0 1];
    %     T_pos_wrt_camera = [R_to_c_from_i zeros(3,1); 0 0 0 1] * T_pos_wrt_imu; 
    %     pos_wrt_camera = T_pos_wrt_camera(1:3,4); 
        pos_wrt_camera = R_to_c_from_i * (pos_wrt_imu(1:3) + [0.25 -2.13 0]') ;
        %pos_wrt_camera(2) = pos_wrt_camera(2) - sin(deg2rad(pitch_start))*pos_wrt_camera(3) + 1.0
        pix = round(cam.KK*(pos_wrt_camera(1:3)/pos_wrt_camera(3)));
        cam_pix(idx,:) = pix(1:2);
        if (pix(1) < 1280 && pix(2) < 960 && pix(1) > 0 && pix(2) >0)
            I(pix(2)-3:pix(2)+3, pix(1)-3:pix(1)+3, 1) = 255;
            I(pix(2)-3:pix(2)+3, pix(1)-3:pix(1)+3, 2) = 0;
            I(pix(2)-3:pix(2)+3, pix(1)-3:pix(1)+3, 3) = 0;
    %         imshow(I)
    %         drawnow;
    %         pause(0.01);
        end
    end
    I = imresize(I,0.5);
    all_frames{start_frame} = I;
    imshow(I);
    drawnow;
end;
% 

% scatter(cam_pix(:,1), cam_pix(:,2));
% xlim([0,1280]);
% ylim([0,960]);
