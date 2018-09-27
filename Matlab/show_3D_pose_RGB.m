close all
clear all
addpath('/home/lin7lr/3Dpose/towards_3D_Human/src')

% videoname = 'S001C002P001R001A057';   
% S_position = strfind(videoname, 'S');
% C_position = strfind(videoname, 'C');
% P_position = strfind(videoname, 'P');
% R_position = strfind(videoname, 'R');
% A_position = strfind(videoname, 'A');
% setupNr = str2num(videoname(S_position+1:S_position+3 ));
% cameraNr = str2num(videoname(C_position+1:C_position+3 ));
% subjectNr = str2num(videoname(P_position+1:P_position+3 ));
% replicationNr = str2num(videoname(R_position+1:R_position+3 ));
% classNr = str2num(videoname(A_position+1:A_position+3 ));

P_Nr = 'P016';
A_Nr= 'A060';

S_Nr = 'S006';
R_Nr = 'R002';
frameid = 40;
%cameraNr = 2;

interaction_flag = 1;

for cameraNr = 1:3
C_Nr = sprintf('C%03d', cameraNr);
    
videoname = [S_Nr,C_Nr,P_Nr,R_Nr,A_Nr]; 

if interaction_flag
    if cameraNr == 2 || cameraNr ==3
        img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera23/interaction/', videoname, '/subject1'];
        img_dir_subject2 = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera23/interaction/', videoname, '/subject2'];
        pose_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/estimated_3dpose_smoothed/',S_Nr,'/camera23/interaction/', videoname];
    else
        img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera1/interaction/', videoname, '/subject1'];
        img_dir_subject2 = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera1/interaction/', videoname, '/subject2'];
        pose_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/estimated_3dpose_smoothed/',S_Nr,'/camera1/interaction/', videoname];
    end

    

else
    if cameraNr == 2 || cameraNr ==3
        img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera23/action/', videoname];
    else
        img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera1/action/', videoname];
    end
    pose_dir = strrep(img_dir, 'cropImg', 'estimated_3dpose_smoothed');
end



all_imgs = dir([img_dir, '/*.png']);

nKeyPoints_RGB = 16;
numImgs = size(all_imgs, 1);
edges_RGB = [1,2;2,3;3,7;7,4;4,5;5,6;11,12;12,13;13,9;9,14;14,15;15,16;7,9;9,10];
colors_RGB = ['k','c','y','y','c','k','b','g','r','r','g','b','m','m'];
LiWi = 2;

fig1 = figure('Name', videoname,'NumberTitle','off','position', [100,600,850, 450]);
set(gcf,'color','w');

       img_filename = sprintf('img_%04d.png', frameid-1);
       %pose_filename = sprintf('Points%d.mat', frameid);
       
       img_path = [img_dir,'/',img_filename ];
        
       im = imread(img_path);
    ax1 = subplot(1,3,1);
    imshow(im)
    
    if interaction_flag 
        pose_filename_subject1 ='pose3d_bbshift_subject1.mat';
        pose_filename_subject2 ='pose3d_bbshift_subject2.mat';
        pose_path_subject1 = [pose_dir,'/',pose_filename_subject1 ];
        pose_path_subject2 = [pose_dir,'/',pose_filename_subject2 ];
        load(pose_path_subject1)
        load(pose_path_subject2)
        notMissingPositions_subject1 =find(~isnan(pose3d_subject1(1,1,:)));
        first_not_missing_frame_pose_subject1 = pose3d_subject1(:,:,notMissingPositions_subject1(1));
        newOrigin_subject1 = (first_not_missing_frame_pose_subject1(7,:) + first_not_missing_frame_pose_subject1(9,:)) / 2;
        pose3d_shifted_subject1 = pose3d_shift(pose3d_subject1,newOrigin_subject1);
        pose3d_shifted_subject2 = pose3d_shift(pose3d_subject2,newOrigin_subject1);
        
        firstFramePoints = pose3d_shifted_subject1(:,:,1);
               %% compute the rotation matrix in xz-plane for all frames
        theta_xz = atan2(firstFramePoints(14,3)- firstFramePoints(13,3), firstFramePoints(14,1)- firstFramePoints(13,1) );
        if theta_xz < 0
           theta_xz = theta_xz + 2*pi;      %  theta :    0 to 2pi
        end
        % a clockwise rotation of theta_xz (around the origin, which is the midpoint of keypoint7 and keypoint9)
        % a clockwise rotation of theta_xz = a counterclockwise rotation of -theta_xz
        R_xz = [cos(-theta_xz) -sin(-theta_xz); sin(-theta_xz) cos(-theta_xz)];
        firstFramePoints_rotated_xz = firstFramePoints;
        firstFramePoints_rotated_xz(:, [1,3]) =  (R_xz * (firstFramePoints_rotated_xz(:, [1,3]))')' ;
        %% compute the rotation matrix in xy-plane for all frames
        theta_xy = atan2(firstFramePoints_rotated_xz(14,2)- firstFramePoints_rotated_xz(13,2), firstFramePoints_rotated_xz(14,1)- firstFramePoints_rotated_xz(13,1) );
        if theta_xy < 0
           theta_xy = theta_xy + 2*pi;      %  theta :    0 to 2pi
        end
        % a clockwise rotation of theta_xy = a counterclockwise rotation of -theta_xy
        R_xy = [cos(-theta_xy) -sin(-theta_xy); sin(-theta_xy) cos(-theta_xy)];    
        firstFramePoints_rotated_xy = firstFramePoints_rotated_xz;
        firstFramePoints_rotated_xy(:, [1,2]) =  (R_xy * (firstFramePoints_rotated_xy(:, [1,2]))')' ;
        %% compute the rotation matrix in yz-plane for all frames
        theta_yz = atan2(firstFramePoints_rotated_xy(13,3)- firstFramePoints_rotated_xy(7,3), firstFramePoints_rotated_xy(13,2)- firstFramePoints_rotated_xy(7,2) );
        if theta_yz < 0
           theta_yz = theta_yz + 2*pi;      %  theta :    0 to 2pi
        end
        % a counterclockwise rotation of (pi-theta_yz)
        R_yz = [cos(pi-theta_yz) -sin(pi-theta_yz); sin(pi-theta_yz) cos(pi-theta_yz)];
        
        pose3d_rotated_subject1 = pose3d_rotation(pose3d_shifted_subject1, R_xz, R_xy, R_yz);
        pose3d_rotated_subject2 = pose3d_rotation(pose3d_shifted_subject2, R_xz, R_xy, R_yz);
        ax2 = subplot(1,3,2);
        %pose3d(isnan(pose3d)) = 0;
        subplot_3DPose(pose3d_subject1(:,:,frameid), nKeyPoints_RGB, edges_RGB, LiWi, colors_RGB )
        hold on
        subplot_3DPose(pose3d_subject2(:,:,frameid), nKeyPoints_RGB, edges_RGB, LiWi, colors_RGB )
        set(gca,'zdir','reverse')
        grid minor
        view([-10,-10,3])
        axis equal
        xlim auto
        ylim auto
        zlim auto
        ax3 = subplot(1,3,3);
        subplot_3DPose(pose3d_rotated_subject1(:,:,frameid), nKeyPoints_RGB, edges_RGB, LiWi, colors_RGB )
        hold on
        subplot_3DPose(pose3d_rotated_subject2(:,:,frameid), nKeyPoints_RGB, edges_RGB, LiWi, colors_RGB )
        set(gca,'zdir','reverse')
        grid minor
        view([-10,-10,3])
        axis equal
        xlim auto
        ylim auto
        zlim auto
        
    else
        pose_filename ='pose3d_bbshift.mat';
        pose_path = [pose_dir,'/',pose_filename ];
        load(pose_path)
        pose3d_shifted = pose3d_shift(pose3d);
        pose3d_rotated = pose3d_rotation(pose3d_shifted);
        ax2 = subplot(1,3,2);
        %pose3d(isnan(pose3d)) = 0;
        subplot_3DPose(pose3d(:,:,frameid), nKeyPoints_RGB, edges_RGB, LiWi, colors_RGB )
        set(gca,'zdir','reverse')
        grid minor
        view([-10,-10,3])
        axis equal
        xlim auto
        ylim auto
        zlim auto
        ax3 = subplot(1,3,3);
        subplot_3DPose(pose3d_rotated(:,:,frameid), nKeyPoints_RGB, edges_RGB, LiWi, colors_RGB )
        set(gca,'zdir','reverse')
        grid minor
        view([-10,-10,3])
        axis equal
        xlim auto
        ylim auto
        zlim auto
    end
    


   
%    title(['setup',num2str(setupNr), ' camera',num2str(cameraNr),' subject',num2str(subjectNr),' replication', num2str(replicationNr),' ', className,  ' frame ', num2str(frameid)])
%    pause(0.01);
%    frames(frameid)=getframe(fig1);
% end
end

% myVideoName = [videoname,'.avi'];
% 
% v = VideoWriter(myVideoName);
% v.Quality = 50;
% dim1 = [];
% if isempty(dim1)
%     v.FrameRate = 2;
% else  % dim1 = 10, only 10 frames are sampled
%     v.FrameRate = 1;
% end
% open(v);
% step = 1;
% for i = 1:step:numFrames
%     writeVideo(v,frames(i))
% end
% close(v);
function pose3d_scalenormed = pose3d_scale_norm(pose3d)

        pose3d_scalenormed = zeros(size(pose3d));
        for frameid = 1:size(pose3d, 3)
            frame_pose = pose3d(:,:,frameid);
            if ~all( frame_pose(:) == 0)
                torso_length = sqrt(sum((frame_pose(7, :) - frame_pose(9, :)).^2));
                frame_pose_scalenormed = frame_pose / torso_length;
                pose3d_scalenormed(:,:,frameid) = frame_pose_scalenormed;
            end
        end
end
function pose3d_shifted = pose3d_shift(pose3d, newOrigin)
        pose3d_shifted = zeros(size(pose3d));
        notMissingPositions =find(pose3d(1,1,:)~=0);
        
        if nargin == 1
            first_not_missing_frame_pose = pose3d(:,:,notMissingPositions(1));
            newOrigin = (first_not_missing_frame_pose(7,:) + first_not_missing_frame_pose(9,:)) / 2;
        end
        
        pose3d_not_missing_shifted = pose3d(:,:,notMissingPositions) - repmat(newOrigin, size(pose3d,1), 1, length(notMissingPositions));
        pose3d_shifted(:,:,notMissingPositions) = pose3d_not_missing_shifted;
        %pose3d_shifted  = pose3d - repmat(newOrigin, size(pose3d,1), 1, size(pose3d,3));
end
function pose3d_rotated = pose3d_rotation(pose3d, R_xz, R_xy, R_yz)

    if nargin ~= 1 && nargin ~= 4
        error('Incorrect number of input arguments!')
    end


    pose3d_rotated = zeros(size(pose3d));
    firstFramePoints = pose3d(:,:,1);
    
    if nargin == 1
        %% compute the rotation matrix in xz-plane for all frames
        theta_xz = atan2(firstFramePoints(14,3)- firstFramePoints(13,3), firstFramePoints(14,1)- firstFramePoints(13,1) );
        if theta_xz < 0
           theta_xz = theta_xz + 2*pi;      %  theta :    0 to 2pi
        end
        % a clockwise rotation of theta_xz (around the origin, which is the midpoint of keypoint7 and keypoint9)
        % a clockwise rotation of theta_xz = a counterclockwise rotation of -theta_xz
        R_xz = [cos(-theta_xz) -sin(-theta_xz); sin(-theta_xz) cos(-theta_xz)];
        firstFramePoints_rotated_xz = firstFramePoints;
        firstFramePoints_rotated_xz(:, [1,3]) =  (R_xz * (firstFramePoints_rotated_xz(:, [1,3]))')' ;
        %% compute the rotation matrix in xy-plane for all frames
        theta_xy = atan2(firstFramePoints_rotated_xz(14,2)- firstFramePoints_rotated_xz(13,2), firstFramePoints_rotated_xz(14,1)- firstFramePoints_rotated_xz(13,1) );
        if theta_xy < 0
           theta_xy = theta_xy + 2*pi;      %  theta :    0 to 2pi
        end
        % a clockwise rotation of theta_xy = a counterclockwise rotation of -theta_xy
        R_xy = [cos(-theta_xy) -sin(-theta_xy); sin(-theta_xy) cos(-theta_xy)];    
        firstFramePoints_rotated_xy = firstFramePoints_rotated_xz;
        firstFramePoints_rotated_xy(:, [1,2]) =  (R_xy * (firstFramePoints_rotated_xy(:, [1,2]))')' ;
        %% compute the rotation matrix in yz-plane for all frames
        theta_yz = atan2(firstFramePoints_rotated_xy(13,3)- firstFramePoints_rotated_xy(7,3), firstFramePoints_rotated_xy(13,2)- firstFramePoints_rotated_xy(7,2) );
        if theta_yz < 0
           theta_yz = theta_yz + 2*pi;      %  theta :    0 to 2pi
        end
        % a counterclockwise rotation of (pi-theta_yz)
        R_yz = [cos(pi-theta_yz) -sin(pi-theta_yz); sin(pi-theta_yz) cos(pi-theta_yz)];
    end
    
    for frameid = 1:size(pose3d, 3)
        frame_pose = pose3d(:,:,frameid);
        if ~all( frame_pose(:) == 0)
            points_rotated_xz = frame_pose;
            points_rotated_xz(:,[1,3]) = (R_xz * (points_rotated_xz(:,[1,3]))')' ;
            points_rotated_xy = points_rotated_xz;
            points_rotated_xy(:, [1,2]) = (R_xy *(points_rotated_xy(:, [1,2]))')'  ;
            points_rotated_yz = points_rotated_xy;
            points_rotated_yz(:, [2,3]) = (R_yz *(points_rotated_yz(:, [2,3]))')'  ;
            pose3d_rotated(:,:,frameid) = points_rotated_yz;
        end
    end
end