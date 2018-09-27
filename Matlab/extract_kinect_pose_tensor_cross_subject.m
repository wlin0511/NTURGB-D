clear all
addpath('/home/lin7lr/3Dpose/towards_3D_Human/src')
addpath('/home/lin7lr/3Dpose/NTU/NTURGB-D-master/Matlab/kinect_pose_normalization')
%% cross-view,  training data: camera2&3    test data: camera1

pose_dir = '/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/kinect_3dpose/cross_subject';

% node = [7 3 2 1 2 3 7 4 5 6 5 4 7 9 10 9 13 12 11 12 13 9 14 15 16 15 14 9 7];  % 29 nodes

node = [1 17 18 19 20 19 18 17 1 13 14 15 16 15 14 13 1 ...
    2 21 3 4 3 21 ...
    9 10 11 12 24 25 12 11 10 9 21 ...
    5 6 7 8 22 23 8 7 6 5 21 2 1 ];

dim1 = 10;
dim2 = length(node)*3;

scale_norm = 1;
shift_norm = 1;
rot_norm = 0;

vel_norm = 1;
acc_norm = 0;
action_sequences_train = dir([pose_dir, '/train/action/S*' ]);
interaction_sequences_train = dir([pose_dir, '/train/interaction/S*' ]);
action_sequences_test = dir([pose_dir, '/test/action/S*' ]);
interaction_sequences_test = dir([pose_dir, '/test/interaction/S*' ]);
tic;
[pose_4d_interaction_train, labels_interaction_train] = pose_tensor_formation_interaction(dim1,dim2, interaction_sequences_train, node,scale_norm, shift_norm, rot_norm, vel_norm, acc_norm);
[pose_4d_action_train, labels_action_train] = pose_tensor_formation_action(dim1,dim2, action_sequences_train, node,scale_norm, shift_norm, rot_norm, vel_norm, acc_norm);
[pose_4d_interaction_test, labels_interaction_test] = pose_tensor_formation_interaction(dim1,dim2, interaction_sequences_test, node, scale_norm, shift_norm,rot_norm, vel_norm, acc_norm);
[pose_4d_action_test, labels_action_test] = pose_tensor_formation_action(dim1,dim2, action_sequences_test, node,scale_norm, shift_norm, rot_norm, vel_norm, acc_norm);
t1= toc

pose_4d_train = cat(4, pose_4d_interaction_train, pose_4d_action_train);
train_labels = [labels_interaction_train; labels_action_train];
pose_4d_test = cat(4, pose_4d_interaction_test, pose_4d_action_test);
test_labels = [labels_interaction_test; labels_action_test];

h5_file_train = '/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/h5/3DPose_kinect/cross_subject/train/train_shifted_scalenorm/train_%dpos.h5';
h5_file_train = sprintf(h5_file_train, length(train_labels));
h5create(h5_file_train, '/dataset', size(pose_4d_train))  % create HDF5 data set
h5write(h5_file_train, '/dataset', pose_4d_train)
h5create(h5_file_train,'/label',size(train_labels),'Datatype','double');
h5write(h5_file_train,'/label',train_labels);

h5_file_test = '/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/h5/3DPose_kinect/cross_subject/test/test_shifted_scalenorm/test_%dpos.h5';
h5_file_test = sprintf(h5_file_test, length(test_labels));
h5create(h5_file_test, '/dataset', size(pose_4d_test))  % create HDF5 data set
h5write(h5_file_test, '/dataset', pose_4d_test)
h5create(h5_file_test,'/label',size(test_labels),'Datatype','double');
h5write(h5_file_test,'/label',test_labels);

function [pose_4d, labels] = pose_tensor_formation_action(dim1,dim2, sequences, node, scale_norm, shift_norm, rot_norm, vel_norm, acc_norm)   % no missing frames
    %% action only contain one subject, filled the other one with zeros
    dim2_interval = 1:dim2;
    numFiles = length(sequences);
    pose_4d = zeros(dim1, dim2 * 2, 3, numFiles);   % two subjects
    labels = zeros(numFiles,1);
    
    for sequenceid = 1:numFiles
        filename = sequences(sequenceid).name(1:20);
        classNr = str2num(filename(18:20));
        disp(['sequenceid: ', num2str(sequenceid), ' filename: ', filename])
        pose_tensor = zeros(dim1, dim2*2, 3); % 2 subjects
        
        sequencePath = [sequences(sequenceid).folder, '/', sequences(sequenceid).name];
        %pose3d_dir = strrep(sequencePath, 'estimated_3dpose', 'estimated_3dpose_smoothed');
        pose3d_dir = sequencePath;
        load([pose3d_dir, '/pose3d.mat'])
        %pose3d = pose3d_smoothed;
        pose3d(isnan(pose3d)) = 0;
        numFrames = size(pose3d, 3);

        step = floor(numFrames/dim1);
        joint_positions = zeros(numFrames, dim2*2);
        %% normalization steps:
        if scale_norm
           pose3d = pose3d_scale_norm(pose3d);
        end
        if shift_norm
           pose3d = pose3d_shift(pose3d);
        end
        if rot_norm
           pose3d = pose3d_rotation(pose3d);
        end
        for frameid = 1:numFrames
            xyz_nodeSequence = pose3d(node,:, frameid)';
            joint_positions(frameid, dim2_interval) = xyz_nodeSequence(:)';
        end
        joint_positions_resized = imresize(joint_positions, [dim1, dim2*2]);
        pose_tensor(:, :, 1) = joint_positions_resized;
        
        for dim1_id = 2:dim1
            if vel_norm
                pose_tensor(dim1_id,:,2) = (pose_tensor(dim1_id,:,1) - pose_tensor(dim1_id-1, :,1))/step;
            else
                pose_tensor(dim1_id,:,2) = pose_tensor(dim1_id,:,1) - pose_tensor(dim1_id-1, :,1);
            end
            if acc_norm
                pose_tensor(dim1_id,:,3) = (pose_tensor(dim1_id,:,2) - pose_tensor(dim1_id-1, :,2))/step;
            else
                pose_tensor(dim1_id,:,3) = pose_tensor(dim1_id,:,2) - pose_tensor(dim1_id-1, :,2);
            end
            
        end
        
        pose_4d(:,:,:,sequenceid) = pose_tensor;
        labels(sequenceid) = classNr; 
    end
    
end

function [pose_4d, labels] = pose_tensor_formation_interaction(dim1,dim2, sequences, node,scale_norm, shift_norm, rot_norm, vel_normalization, acc_normalization)   % there might be missing frames
    %% action only contain one subject, the other

    numFiles = length(sequences);
    pose_4d = zeros(dim1, dim2 * 2, 3, numFiles);   % two subjects
    labels = zeros(numFiles,1);
    
    for sequenceid = 1:numFiles
        filename = sequences(sequenceid).name(1:20);
        classNr = str2num(filename(18:20));
        disp(['sequenceid: ', num2str(sequenceid), ' filename: ', filename])
        pose_tensor = zeros(dim1, dim2*2, 3); % 2 subjects
        
        sequencePath = [sequences(sequenceid).folder, '/', sequences(sequenceid).name];
        %pose3d_dir = strrep(sequencePath, 'estimated_3dpose', 'estimated_3dpose_smoothed');
        pose3d_dir = sequencePath;
        load([pose3d_dir, '/pose3d_subject1.mat'])
        load([pose3d_dir, '/pose3d_subject2.mat'])
%         pose3d_subject1 = pose3d_subject1_smoothed;
%         pose3d_subject2 = pose3d_subject2_smoothed;        
        pose3d_subject1(isnan(pose3d_subject1)) = 0;
        pose3d_subject2(isnan(pose3d_subject2)) = 0;
        numFrames = size(pose3d_subject1, 3);
        
        step = floor(numFrames/dim1);
        joint_positions = zeros(numFrames, dim2*2);
        %% normalization steps:
        if scale_norm
            % the torso length is length of limb between point 7 and point 9
           pose3d_subject1 = pose3d_scale_norm(pose3d_subject1);
           pose3d_subject2 = pose3d_scale_norm(pose3d_subject2);
        end
         if shift_norm
            notMissingPositions =find(pose3d_subject1(1,1,:)~=0);    
            first_not_missing_frame_pose = pose3d_subject1(:,:,notMissingPositions(1));
            newOrigin = first_not_missing_frame_pose(2,:) ;
            
           pose3d_subject1 = pose3d_shift(pose3d_subject1, newOrigin);
           pose3d_subject2 = pose3d_shift(pose3d_subject2, newOrigin);
        end
        if rot_norm
            firstFramePoints = pose3d_subject1(:,:,1);
           %% compute the rotation matrix in xz_plane     left shoulder -> right shoulder  parallel to x axis (positive direction)
            theta_xz = atan2(firstFramePoints(5,3)- firstFramePoints(9,3), firstFramePoints(5,1)- firstFramePoints(9,1) );
            if theta_xz < 0
               theta_xz = theta_xz + 2*pi;      %  theta :    0 to 2pi
            end
            % a clockwise rotation of theta_xz (around the origin, which is the midpoint of keypoint7 and keypoint9)
            % a clockwise rotation of theta_xz = a counterclockwise rotation of -theta_xz
            R_xz = [cos(-theta_xz) -sin(-theta_xz); sin(-theta_xz) cos(-theta_xz)];
            firstFramePoints_rotated_xz = firstFramePoints;
            firstFramePoints_rotated_xz(:, [1,3]) =  (R_xz * (firstFramePoints_rotated_xz(:, [1,3]))')' ;
            %% compute the rotation matrix in xy_plane   left shoulder -> right shoulder 
            theta_xy = atan2(firstFramePoints_rotated_xz(5,2)- firstFramePoints_rotated_xz(9,2), firstFramePoints_rotated_xz(5,1)- firstFramePoints_rotated_xz(9,1) );
            if theta_xy < 0
               theta_xy = theta_xy + 2*pi;      %  theta :    0 to 2pi
            end
            % a clockwise rotation of theta_xy = a counterclockwise rotation of -theta_xy
            R_xy = [cos(-theta_xy) -sin(-theta_xy); sin(-theta_xy) cos(-theta_xy)];    
            firstFramePoints_rotated_xy = firstFramePoints_rotated_xz;
            firstFramePoints_rotated_xy(:, [1,2]) =  (R_xy * (firstFramePoints_rotated_xy(:, [1,2]))')' ;
            %% compute the rotation matrix in yz-plane   right shoulder -> belly
            theta_yz = atan2(firstFramePoints_rotated_xy(9,3)- firstFramePoints_rotated_xy(1,3), firstFramePoints_rotated_xy(9,2)- firstFramePoints_rotated_xy(1,2) );
            if theta_yz < 0
               theta_yz = theta_yz + 2*pi;      %  theta :    0 to 2pi
            end
            % a counterclockwise rotation of (pi-theta_yz)
            R_yz = [cos(pi-theta_yz) -sin(pi-theta_yz); sin(pi-theta_yz) cos(pi-theta_yz)];
           pose3d_subject1 = pose3d_rotation(pose3d_subject1, R_xz, R_xy, R_yz);
           pose3d_subject2 = pose3d_rotation(pose3d_subject2, R_xz, R_xy, R_yz);
        end
        for frameid = 1:numFrames

            
            xyz_nodeSequence_subject1 = pose3d_subject1(node,:,frameid)';
            xyz_nodeSequence_subject2 = pose3d_subject2(node,:,frameid)';
                        
            joint_positions(frameid, 1:dim2) = xyz_nodeSequence_subject1(:)';           
            joint_positions(frameid, dim2+1:2*dim2) = xyz_nodeSequence_subject2(:)';           
        end     
        joint_positions_resized = imresize(joint_positions, [dim1, dim2*2]);
        pose_tensor(:, :, 1) = joint_positions_resized;        
        for dim1_id = 2:dim1
            if vel_normalization
                pose_tensor(dim1_id,:,2) = (pose_tensor(dim1_id,:,1) - pose_tensor(dim1_id-1, :,1))/step;
            else
                pose_tensor(dim1_id,:,2) = pose_tensor(dim1_id,:,1) - pose_tensor(dim1_id-1, :,1);
            end
            if acc_normalization
                pose_tensor(dim1_id,:,3) = (pose_tensor(dim1_id,:,2) - pose_tensor(dim1_id-1, :,2))/step;
            else
                pose_tensor(dim1_id,:,3) = pose_tensor(dim1_id,:,2) - pose_tensor(dim1_id-1, :,2);
            end            
        end        
        pose_4d(:,:,:,sequenceid) = pose_tensor;
        labels(sequenceid) = classNr; 
    end
    
end




function show3Dpose()

    nKeyPoints = 16;
    LiWi = 3;
    edges = [1,2;2,3;3,7;7,4;4,5;5,6;11,12;12,13;13,9;9,14;14,15;15,16;7,9;9,10];
    colors = ['k','c','y','y','c','k','b','g','r','r','g','b','m','m'];
    
    figure
    subplot_3DPose(pose3d(:,:,1), nKeyPoints, edges, LiWi, colors )
    figure
    subplot_3DPose(pose3d_rotated(:,:,1), nKeyPoints, edges, LiWi, colors )
    
    
     numJoints = 25;
    LiWi = 3;
    edges = [1,17;1,13;17,18;13,14;18,19;14,15;19,20;15,16;1,2;2,21;21,9;21,5;21,3;3,4;9,10;5,6;10,11;6,7;11,12;7,8;12,24;8,22;12,25;8,23  ];
    colors = ['y','y',  'b',   'b', 'k',  'k',  'y',  'y', 'm','k',  'm', 'm','b', 'r', 'g', 'g', 'c', 'c', 'k','k','r',  'r', 'b','b'  ];
    figure
    plot_3DPose(pose3d_subject1(:,:,1), numJoints, edges, LiWi, colors )
    hold on
    plot_3DPose(pose3d_subject2(:,:,1), numJoints, edges, LiWi, colors )
       view([-0,1,-3])
   axis equal
      xlabel('x')
   ylabel('y')
   zlabel('z')
   set(gca,'Xdir','reverse')
end