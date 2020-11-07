%% Step1
clear all;


frame_rate = 30;


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
index=find(signal(:,1)>19);
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
% time=time(1:5);

len_pre = 5;%sec
len_post = 15;%sec
% 0to50 or 0to80 in5s: 5 to 10 (15 in total)
% pre5: 5 to 10 (15 in total)
% pre30: -25 to 40 (15 in total)
% pre30: 5 to 10 (15 in total) for initial response

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

%% split video for eyemovent files (into 5 video files)
clear all;

len_frame=40;%frame
num_cycle=20;
num_total=109;
frame_rate=40;%frame/sec

[filename, path]=uigetfile('.avi');
cd(path);

v=VideoReader(filename);

for i=1:num_total/num_cycle
    i_frame=0;
    while(i_frame < num_cycle*len_frame)
        vidFrames(:,:,:,i_frame+1) = readFrame(v);
        %         image(vidFrames(:,:,:,j));
        %         pause(1/v.FrameRate);
        i_frame=i_frame+1;
    end
    
    vw = VideoWriter([filename,'_',num2str(i),'.avi']);
    vw.FrameRate=frame_rate;
    open(vw)
    writeVideo(vw,vidFrames);
    close(vw)
end


%% transport d to some temporal solution
clear;
load('matlab.mat');
frame_rate =14;
time_solution = 0.5;
num_bin = ceil(length(d(:,1))/frame_rate/time_solution);
len_mean = time_solution/(1/frame_rate);

for i=1:num_bin
    start_n = (i-1)*len_mean+1;
    end_n = i*len_mean;
    if end_n>length(d(:,1))
        end_n=length(d(:,1));
    end
    d_f(i,:)=mean(d(start_n:end_n,:),1);
    
end

num_file = size(d_f,2);

df_m = mean(d_f,2);
df_mf = mean(d_f(:,1:round(num_file/2)),2);
df_ms = mean(d_f(:,round(num_file/2)+1:end),2);
df_con = mean(mean(d_f(1:num_bin/3,:)));
x=0:time_solution:time_solution*num_bin;x=x(2:end);
% figure;plot(x,df_m,'b');hold on;plot(x,df_mf,'r');plot(x,df_ms,'g');line(x,df_con*ones(num_bin));grid on;
figure;plot(x,df_m,'b');hold on;plot(x,d_f(:,1),'r');plot(x,d_f(:,5),'g');line(x,df_con*ones(num_bin));grid on;
%% calculate and transport p to dis in some temporal solution
% load('p.mat');
frame_rate =25;
time_solution = 0.2;
num_bin = ceil(length(p(:,1))/frame_rate/time_solution);
len_mean = time_solution/(1/frame_rate);

r=[48,-184];
d=sqrt((p(:,2)-r(1)).^2+(p(:,3)-r(2)).^2);

for i=1:num_bin
    start_n = (i-1)*len_mean+1;
    end_n = i*len_mean;
    if end_n>length(d(:,1))
        end_n=length(d(:,1));
    end
    dp_f(i,:)=mean(d(start_n:end_n,:),1);
    
end

x=0:time_solution:time_solution*num_bin;x=x(2:end);
figure;plot(x,dp_f,'b');grid on;
xlim([0, num_bin*time_solution]);
%%
figure;
plot(x,mean(LSdf,2),'r');
hold on
plot(x,mean(TSdf,2),'b');
figure;
plot(x,mean(LSdf2,2),'r');
hold on
plot(x,mean(TSdf2,2),'b');
figure;
plot(x,mean(LSdf3,2),'r');
hold on
plot(x,mean(TSdf3,2),'b');
%%
figure;
plot(x,LSdf(:,1),'r');
hold on
plot(x,TSdf(:,1),'b');
figure;
plot(x,LSdf2(:,1),'r');
hold on
plot(x,TSdf2(:,1),'b');
figure;
plot(x,LSdf3(:,1),'r');
hold on
plot(x,TSdf3(:,1),'b');