%% Sam Cooney Lab 2
clc;clear;close all;
published = datestr(now,21)

%% Procedure 1
%% Part A
clc;clear;close all;
fs = 100;Ts = 1/fs;t = (0:Ts:110);
f1 = 20;w1 = 2*pi*f1;
x1 = cos(w1.*t);
figure(1)
plot(t,x1)
axis([0 0.5 -1 1])
xlabel('Time (s)')
ylabel('x1')
title('Plot of a Sinusoid, fs = 100Hz, f1 = 20Hz')
figure(2)
spectrogram(x1,256,250,512,fs,'yaxis')
title('Spectrogram of a Sinusoid')

%% Part B
f2 = 90;w2 = 2*pi*f2;
x2 = cos(w2.*t);
figure(3)
plot(t,x2)
axis([0 0.52 -1 1])
xlabel('Time (s)')
ylabel('x2')
title('Plot of a Sinusoid, fs = 100Hz, f1 = 90Hz')
figure(4)
spectrogram(x2,256,250,512,fs,'yaxis')
title('Spectrogram of a Sinusoid')

%% Part C
clc;clear;close all;
fs = 1000;Ts = 1/fs;t = (0:Ts:110);
f2 = 90;w2 = 2*pi*f2;
x2 = cos(w2.*t);
figure(3)
plot(t,x2)
axis([0 0.52 -1 1])
xlabel('Time (s)')
ylabel('x1')
title('Plot of a Sinusoid, fs = 1000Hz, f1 = 90Hz')
figure(4)
spectrogram(x2,256,250,512,fs,'yaxis')
title('Spectrogram of a Sinusoid')

%% Procedure 2
%% Part A
clc;clear;close all;
load('lighthouse.mat')
show_img(xx,1)
title('Original')
xx3 = xx(1:3:end,1:3:end);
show_img(xx3,2)
title('Downsized Image')

%% Part B
% Part i
xxstr = zeros(length(xx(:,1)),length(xx));
xxstr(1:3:end,1:3:end) = xx3;
show_img(xxstr,3)
title('Upsized Image Using Stretching')

% Part ii
L1 = length(xx3);L2 = length(xx3(:,1));
nn = ceil((0.9:1:3*L1)/3);mm = ceil((0.9:1:3*L2)/3);
xx3hold = xx3(mm,nn);
show_img(xx3hold,4)
title('Upsized Image (zero-order hold)')
