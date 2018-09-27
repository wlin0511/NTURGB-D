close all
clear all
addpath('/home/lin7lr/3Dpose/towards_3D_Human/src')

P_Nr = 'P028';
A_Nr= 'A055';
visualization_frameid = 35;
S_Nr = 'S013';
R_Nr = 'R001';

nKeypoints = 25;
for cameraNr = 1:3
    C_Nr = sprintf('C%03d', cameraNr);
    
    videoname = [S_Nr,C_Nr,P_Nr,R_Nr,A_Nr]; 
    if cameraNr == 2 || cameraNr ==3
        img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera23/action/', videoname];
        if ~exist(img_dir,'dir')
            img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera23/interaction/', videoname, '/subject1'];
        end
    else
        img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera1/action/', videoname];
        if ~exist(img_dir,'dir')
            img_dir = ['/mnt/Projects/CV-008_Students/ActionRecognitionProjects/TrainingData/NTU/cropImg/',S_Nr,'/camera1/interaction/', videoname, '/subject1'];
        end
    end
    
    
    %videoname = 'S004C001P003R001A043';


    img_filename = sprintf('img_%04d.png', visualization_frameid);
    pose_filename = sprintf('Points%d.mat', visualization_frameid+1);
    S_Nr = videoname(1:4);
    
    pose_dir = strrep(img_dir, 'cropImg', 'estimated_3dpose');
    img_path = [img_dir,'/',img_filename ];
    pose_path = [pose_dir,'/',pose_filename ];

    S_position = strfind(videoname, 'S');
    C_position = strfind(videoname, 'C');
    P_position = strfind(videoname, 'P');
    R_position = strfind(videoname, 'R');
    A_position = strfind(videoname, 'A');
%     setupNr = str2num(videoname(S_position+1:S_position+3 ));
%     %cameraNr = str2num(videoname(C_position+1:C_position+3 ));
%     subjectNr = str2num(videoname(P_position+1:P_position+3 ));
%     replicationNr = str2num(videoname(R_position+1:R_position+3 ));
    classNr = str2num(videoname(A_position+1:A_position+3 ));
    switch classNr
        case 1, className = 'drink water';case 2, className = 'eat meal/snack';case 3, className = 'brushing teeth';case 4, className = 'brushing hair';case 5, className = 'drop';
        case 6, className = 'pickup';case 7, className = 'throw';case 8, className = 'sit down';case 9, className = 'stand up';case 10, className = 'clap';
        case 11, className = 'read';case 12, className = 'write'; case 13, className = 'tear up paper';case 14, className = 'wear jacket'; case 15, className = 'take off jacket';
        case 16, className = 'wear a shoe'; case 17, className = 'take off a shoe'; case 18, className = 'wear glasses';case 19, className = 'take off glasses'; case 20, className = 'put on hat/cap';
        case 21, className = 'take off hat/cap';case 22, className = 'cheer up'; case 23, className = 'hand waving';case 24, className = 'kick sth'; case 25, className = 'put inside/take out fr pocket';
        case 26, className = 'hop'; case 27, className = 'jump up'; case 28, className = 'make phone call/answer phone';case 29, className = 'play with phone/tablet'; case 30, className = 'type on keyboard';
        case 31, className = 'point at sth';case 32, className = 'take a selfie'; case 33, className = 'check time';case 34, className = 'tub two hands together'; case 35, className = 'node head/bow';
        case 36, className = 'shake head'; case 37, className = 'wipe face'; case 38, className = 'salute';case 39, className = 'put the palms together'; case 40, className = 'cross hands in front';
        case 41, className = 'sneeze/cough';case 42, className = 'stagger'; case 43, className = 'fall';case 44, className = 'touch head(headache)'; case 45, className = 'touch chest(stomachache)';
        case 46, className = 'touch back(backache)'; case 47, className = 'touch neck(neckache)'; case 48, className = 'nausea or vomit';case 49, className = 'use a fan with hand or paper/feel warm'; case 50, className = 'punch/slap the other';
        case 51, className = 'kick the other';case 52, className = 'push the other'; case 53, className = 'pat on the back';case 54, className = 'point finger at the other'; case 55, className = 'hug the other';
        case 56, className = 'give sth to the other'; case 57, className = 'touch the others pocket'; case 58, className = 'handshake';case 59, className = 'walk towards each other'; case 60, className = 'walk apart fr each other';
        otherwise, error('invalid classid!')
    end
    skeletonfilename = ['/mnt/Projects/CV-008_Students/actionRecognition/NTU-RGB-D/nturgb+d_skeletons/', videoname, '.skeleton'];

    bodyinfo = read_skeleton_file(skeletonfilename);   % bodyinfo(f).bodies(b).joints(j)
    LiWi = 2;

    numJoints_kinect = 25;
    edges_kinect = [1,17;1,13;17,18;13,14;18,19;14,15;19,20;15,16;1,2;2,21;21,9;21,5;21,3;3,4;9,10;5,6;10,11;6,7;11,12;7,8;12,24;8,22;12,25;8,23  ];
    colors_kinect = ['y','y',  'c',   'c', 'k',  'k',  'y',  'y', 'm','m',  'r', 'r','m', 'm', 'g', 'g', 'b', 'b', 'k','k','r',  'r', 'r','r'  ];

    nKeyPoints_RGB = 16;
    edges_RGB = [1,2;2,3;3,7;7,4;4,5;5,6;11,12;12,13;13,9;9,14;14,15;15,16;7,9;9,10];
    colors_RGB = ['k','c','y','y','c','k','b','g','r','r','g','b','m','m'];
    numFrames = length(bodyinfo);
    num_bodies_all_frames = zeros(1,numFrames);
    for frameid = 1: numFrames
        num_bodies_all_frames(frameid) = length(bodyinfo(frameid).bodies);
    end
    highest_num_bodies = max(num_bodies_all_frames);
    if highest_num_bodies == 0
       error('No subject!!')   
    end
    
    %% recorded all the bodyIDs that appeared  
    bodyIDs = strings( sum(num_bodies_all_frames),1 );
    counter = 0;
    for frameid = 1: numFrames
        numBodies = num_bodies_all_frames(frameid);
        if numBodies ~= 0
            for bodyid = 1:numBodies
                counter = counter +1;
                bodyID = bodyinfo(frameid).bodies(bodyid).bodyID;
                bodyIDs(counter,1) = bodyID;
            end
        end
    end            
    bodyIDs = unique(bodyIDs);
    num_bodyIDs = length(bodyIDs);
    disp([videoname, ' has ', num2str(num_bodyIDs),' bodyIDs!!'])
        
        
     %% tracking the pose with the given bodyIDs, every bodyID corresponds to an independent subject
     pose3D = NaN(nKeypoints, 3, numFrames, num_bodyIDs);
     for frameid = 1:numFrames
            numBodies = num_bodies_all_frames(frameid);
            for bodyid = 1:numBodies
                bodyID = bodyinfo(frameid).bodies(bodyid).bodyID;                    
                body_corresponding_id = find (bodyIDs == num2str(bodyID));
                cell_array = struct2cell(bodyinfo(frameid).bodies(bodyid).joints);
                pose3D(:,:,frameid, body_corresponding_id) = cell2mat(reshape(cell_array(1:3,:,:), 3, 25)');          
            end
     end

    if num_bodyIDs==1   % all the frames have only one subject   
        xmax = max(max(pose3D(:,1,:)));
        xmin = min(min(pose3D(:,1,:)));
        ymax = max(max(pose3D(:,2,:)));
        ymin = min(min(pose3D(:,2,:)));
        zmax = max(max(pose3D(:,3,:)));
        zmin = min(min(pose3D(:,3,:)));
    else   % multiple subjects
        xmax = max(max(max(pose3D(:,1,:,:))));
        xmin = min(min(min(pose3D(:,1,:,:))));
        ymax = max(max(max(pose3D(:,2,:,:))));
        ymin = min(min(min(pose3D(:,2,:,:))));
        zmax = max(max(max(pose3D(:,3,:,:))));
        zmin = min(min(min(pose3D(:,3,:,:))));
    end 

    fig1 = figure('Name', videoname,'NumberTitle','off','position', [100,600,850, 450]);
    set(gcf,'color','w');
    im = imread(img_path);
    ax1 = subplot(1,3,1);
    imshow(im)
    ax2 = subplot(1,3,2);
    %set(gca,'Color',256/256 * [1,1,1])
    %set(gca,'xtick',[-0.5:0.1:0.5])
       %numBodies = length(bodyinfo(frameid).bodies); 
       if highest_num_bodies==1    % all the frames have only one subject
           plot_3DPose(pose3D(:,:,visualization_frameid), numJoints_kinect, edges_kinect, LiWi, colors_kinect )
       else   % multiple subjects
           numBodies = num_bodies_all_frames(visualization_frameid);
           %numBodies = 1
           for bodyid = 1:numBodies
               plot_3DPose(pose3D(:,:,visualization_frameid, bodyid), numJoints_kinect, edges_kinect, LiWi, colors_kinect )
               hold on
           end
       end
    % view([-0,1,-3])
    % rotate(ax2, [0,1,0], 20)
       axis equal
    %    xlim([xmin xmax])
    %    ylim([ymin ymax])
    %    zlim([zmin zmax])
       xlabel('x')
       ylabel('z')
       zlabel('y')
       %set(gca,'Xdir','reverse')
       grid minor
    ax3 = subplot(1,3,3);

    load(pose_path)
    subplot_3DPose(x, nKeyPoints_RGB, edges_RGB, LiWi, colors_RGB )
     set(gca,'zdir','reverse')
    grid minor
    %view([-0,-1,-3])

    camroll(10)
    rotate3d on
   %title([videoname,  ' frame ', num2str(visualization_frameid)])
end





