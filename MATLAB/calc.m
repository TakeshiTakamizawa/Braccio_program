clc; clear all; close all;

th1 = deg2rad(30);
th2 = deg2rad(30);
th3 = deg2rad(30);
th4 = deg2rad(30);
th5 = deg2rad(0);
th6 = deg2rad(0);

l0 = 10;
l1 = 10;
l2 = 10;
l3 = 10;
l4 = 10;
l5 = 10;
l6 = 0;

%% σを用いた場合(仕様書遵守)
% calculation
sigma4 = cos(th2)*sin(th1)*sin(th3) + cos(th3)*sin(th1)*sin(th2);

sigma5 = cos(th1)*cos(th2)*sin(th3) + cos(th1)*cos(th3)*sin(th2);

sigma6 = cos(th2)*cos(th3) - sin(th2)*sin(th3);


sigma1 = cos(th4)*sigma4 ...
    - sin(th4)*( sin(th1)*sin(th2)*sin(th3) ...
    - cos(th2)*cos(th3)*sin(th1) );

sigma2 = cos(th4)*sigma5 ...
    - sin(th4)*( cos(th1)*sin(th2)*sin(th3) ...
    - cos(th1)*cos(th2)*cos(th3) );

sigma3 = cos(th4)*sigma6 ...
    - sin(th4)*( cos(th2)*sin(th3) ...
    + cos(th3)*sin(th2) );

P = [
    l3*sigma4 + l4*sigma1 + l5*sigma1 + l2*sin(th1)*sin(th2);

   -l3*sigma5 - l4*sigma2 - l5*sigma2 - l2*cos(th1)*sin(th2);

    l0 + l1 + l2*cos(th2) + l4*sigma3 + l5*sigma3 + l3*sigma6
];

disp("σを用いた")
fprintf("x = %.3f\n", P(1));
fprintf("y = %.3f\n", P(2));
fprintf("z = %.3f\n", P(3));

%% th6までで回転行列

% z軸回転
R01=[cos(th1) -sin(th1) 0;
     sin(th1) cos(th1) 0;
     0 0 1];

R45=[cos(th5) -sin(th5) 0;
     sin(th5) cos(th5) 0;
     0 0 1];

% x軸回転
R12=[1 0 0;
     0 cos(th2) -sin(th2);
     0 sin(th2) cos(th2)];

R23=[1 0 0;
     0 cos(th3) -sin(th3);
     0 sin(th3) cos(th3)];

R34=[1 0 0;
     0 cos(th4) -sin(th4);
     0 sin(th4) cos(th4)];

% y軸回転
R56=[cos(th6) 0 sin(th6);
     0 1 0;
     -sin(th6) 0 cos(th6)];

% 各リンクベクトル
L0=[0;0;l0];
L1=[0;0;l1];
L2=[0;0;l2];
L3=[0;0;l3];
L4=[0;0;l4];
L5=[0;0;l5];
L6=[0;0;l6];

% 順運動学計算
P0r = ...
R01*R12*R23*R34*R45*R56*L6 ...
+R01*R12*R23*R34*R45*L5 ...
+ R01*R12*R23*R34*L4 ...
+ R01*R12*R23*L3 ...
+ R01*R12*L2 ...
+ R01*L1 ...
+ L0;

% 表示
disp('th6まで導出式から求めた位置ベクトル')
fprintf('x=%.4f\n',P0r(1))
fprintf('y=%.4f\n',P0r(2))
fprintf('z=%.4f\n',P0r(3))