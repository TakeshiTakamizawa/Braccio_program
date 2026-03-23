clear all
close all
clc
setappdata(0,'patch_h',[]);
num = 30; % home to start, and end to home ani steps
meth = 3;

movie_num = 0; % < 1:録画（ファイル名に注意！）

filename = sprintf('test2.mp4');
if movie_num == 1
    vid = VideoWriter(filename, 'MPEG-4');
    vid.FrameRate = 10;
    open(vid);
end

for t = 0:.1:5*pi
    if meth == 1
        Px = 30*t*sin(t);
        Py = 900;
        Pz = 30*t*cos(t);
    elseif meth == 2 % ハート
        scale = 30;
        Px = scale*(16*(sin(t))^3);
        Py = 900;
        Pz = scale*(13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t));
    elseif meth == 3
        [Px, Pz] = happy_haruharu(t);
        Py = 900;
    end

    [theta1,theta2,theta3,theta4,theta5,theta6] = pumaIK(Px,Py,Pz);
    if t==0 %move to start of demo
        pumaANI(theta1,theta2,theta3,theta4,theta5,theta6,num,'n');
    end
    % Theta 4, 5 & 6 are zero due to plotting at wrist origen.
    pumaANI(theta1,theta2,theta3,0,0,0,num,'y');
    if movie_num == 1
        frame = getframe(gcf);
        writeVideo(vid, frame);
    end
end
if movie_num == 1
    close(vid);
end
pumaANI(90,-90,-90,0,0,0,num,'n'); %ホームポジションに戻る　まっすぐな姿勢
