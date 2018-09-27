
addpath('/home/lin7lr/3Dpose/NTU/NTURGB-D-master/Matlab')
% noisy_detection_list = '/mnt/Projects/CV-008_Students/actionRecognition/NTU-RGB-D/noisy_detection_list.txt';
% fileID1 = fopen(noisy_detection_list, 'w');
filename = 'S001C002P003R002A056';

skeletonFileDir = '/mnt/Projects/CV-008_Students/actionRecognition/NTU-RGB-D/nturgb+d_skeletons/';


skeletonfilename = [skeletonFileDir, filename, '.skeleton' ];

%for sequenceid = 1: length(sequences)
    
    %filename = sequences(sequenceid).name(1:20);
    classNr = str2num(filename(18:20));
    %disp(['sequenceid: ', num2str(sequenceid), ' filename: ', filename])
    %skeletonfilename = [sequences(sequenceid).folder,'/', sequences(sequenceid).name];
    bodyinfo = read_skeleton_file(skeletonfilename);
    bodyIDs = [];
    numFrames = length(bodyinfo);
    
        num_bodies_before_denoising = zeros(1,numFrames);
        %counter = 0;
        for frameid = 1: numFrames
            num_bodies_before_denoising(frameid) = length(bodyinfo(frameid).bodies);
            if num_bodies_before_denoising(frameid) ~= 0
                for bodyid = 1:num_bodies_before_denoising(frameid)
                    %counter = counter + 1;
                    bodyIDs = [bodyIDs, bodyinfo(frameid).bodies(bodyid).bodyID];   % bodyID is int64
                end
            end
        end
        unique(bodyIDs)
    %matlab.lang.makeUniqueStrings(bodyIDs)
    %% kinect's body tracker is prone to detecting some objects, e.g. seats or tables as bodies. To filter out the these noisy detections, for each tracked skeleton we calculate the spread of the joint locations towards...
    spread_ratio
    for frameid = 1: numFrames
        if num_bodies_before_denoising(frameid) == 0   % if this frame is missing in detection
            continue
        else
            for bodyid = 1:num_bodies_before_denoising(frameid)
                %struct2cell(bodyinfo(frameid).bodies(bodyid).joints.x)
                A = [bodyinfo(frameid).bodies(bodyid).joints(:).x; bodyinfo(frameid).bodies(bodyid).joints(:).y;bodyinfo(frameid).bodies(bodyid).joints(:).z]
            end
        end
        
    end
    
    
    t = 1
    
%end
