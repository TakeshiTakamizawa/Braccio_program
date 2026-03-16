clc; clear; close all;

%% link length
l0=0.1; l1=0.1; l2=0.1; l3=0.1; l4=0.1; l5=0.1; l6=0.1;

%% 目標位置
P_target = [0.2;0.1;0.4];

%% 初期角度
th = deg2rad([10 10 10 10 10 10])';

%% iteration
for k = 1:1000

    th1=th(1);
    th2=th(2);
    th3=th(3);
    th4=th(4);
    th5=th(5);
    th6=th(6);

    %% 回転行列
    R01=[cos(th1) -sin(th1) 0;
         sin(th1) cos(th1) 0;
         0 0 1];

    R12=[1 0 0;
         0 cos(th2) -sin(th2);
         0 sin(th2) cos(th2)];

    R23=[1 0 0;
         0 cos(th3) -sin(th3);
         0 sin(th3) cos(th3)];

    R34=[1 0 0;
         0 cos(th4) -sin(th4);
         0 sin(th4) cos(th4)];

    R45=[cos(th5) -sin(th5) 0;
         sin(th5) cos(th5) 0;
         0 0 1];

    R56=[cos(th6) 0 sin(th6);
         0 1 0;
         -sin(th6) 0 cos(th6)];

    %% link vector
    L0=[0;0;l0];
    L1=[0;0;l1];
    L2=[0;0;l2];
    L3=[0;0;l3];
    L4=[0;0;l4];
    L5=[0;0;l5];
    L6=[0;0;l6];

    %% forward kinematics
    P = ...
    R01*R12*R23*R34*R45*R56*L6 ...
    +R01*R12*R23*R34*R45*L5 ...
    +R01*R12*R23*R34*L4 ...
    +R01*R12*R23*L3 ...
    +R01*R12*L2 ...
    +R01*L1 ...
    +L0;

    %% 誤差
    e = P_target - P;

    if norm(e) < 1e-5
        break
    end

    %% 数値ヤコビアン
    J=zeros(3,6);
    d=1e-6;

    for i=1:6

        th_temp=th;
        th_temp(i)=th_temp(i)+d;

        th1=th_temp(1);
        th2=th_temp(2);
        th3=th_temp(3);
        th4=th_temp(4);
        th5=th_temp(5);
        th6=th_temp(6);

        R01=[cos(th1) -sin(th1) 0;
             sin(th1) cos(th1) 0;
             0 0 1];

        R12=[1 0 0;
             0 cos(th2) -sin(th2);
             0 sin(th2) cos(th2)];

        R23=[1 0 0;
             0 cos(th3) -sin(th3);
             0 sin(th3) cos(th3)];

        R34=[1 0 0;
             0 cos(th4) -sin(th4);
             0 sin(th4) cos(th4)];

        R45=[cos(th5) -sin(th5) 0;
             sin(th5) cos(th5) 0;
             0 0 1];

        R56=[cos(th6) 0 sin(th6);
             0 1 0;
             -sin(th6) 0 cos(th6)];

        P2 = ...
        R01*R12*R23*R34*R45*R56*L6 ...
        +R01*R12*R23*R34*R45*L5 ...
        +R01*R12*R23*R34*L4 ...
        +R01*R12*R23*L3 ...
        +R01*R12*L2 ...
        +R01*L1 ...
        +L0;

        J(:,i)=(P2-P)/d;

    end

    %% Newton update
    th = th + pinv(J)*e;

end

%% 結果
disp("joint angle (deg)")
disp(rad2deg(th))