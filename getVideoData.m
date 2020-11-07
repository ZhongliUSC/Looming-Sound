%% Step1
clear all;


frame_rate = 30;% input the frame rate of video


% Load the data file
[filename, path]=uigetfile('.mp4');
cd(path);

v=VideoReader(filename);
v.CurrentTime=10;

vidFrame = readFrame(v);
imagesc(vidFrame);
bw=roipoly;


v=VideoReader(filename);

i=1;
while hasFrame(v)
    vidFrame = readFrame(v);
    vg=rgb2gray(vidFrame);
    signal(i,1)=mean(vg(bw));
    signal(i,2)=v.CurrentTime;
    i=i+1;
end

figure;plot(signal(:,1));

%% Step2
threshold_value=20;% input threshold value to identify the onset of stimuli
index=find(signal(:,1)>threshold_value);
index=[30*(3*60+27)];
len_ISI = 20;

j=2;
index2=index(1);
for i=2:length(index)
    if index(i)-index(i-1)>=(len_ISI-2)*frame_rate
        index2(j)=index(i);
        j=j+1;
    end
end

time = signal(index2,2);

length(time)

%% Step3

% input the length of video
len_pre = 5;%sec
len_post = 15;%sec

for i=1:length(time)
    v.CurrentTime = time(i)-len_pre;
    j=1;
    while(v.CurrentTime <= time(i)+len_post)
        vidFrames(:,:,:,j) = readFrame(v);
%         image(vidFrames(:,:,:,j));
%         pause(1/v.FrameRate);
        j=j+1; 
    end
    
    vw = VideoWriter([filename,'_',num2str(i),'.avi']);
    vw.FrameRate=frame_rate;
    open(vw)
    writeVideo(vw,vidFrames);
    close(vw)
    
    close all;
    clear vidFrames;
    
end
