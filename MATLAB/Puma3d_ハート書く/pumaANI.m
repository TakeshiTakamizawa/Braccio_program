function [px,py,pz]=pumaANI(theta1,theta2,theta3,theta4,theta5,theta6,n,trail)
    % This function animates the PUMA 762 robot for given joint angles.
    % n = number of steps
    % trail = 'y' or 'n' (y = leave trail)

    % --- D-H Parameters ---
    a2 = 650; a3 = 0; d3 = 190; d4 = 600;

    % --- Load previous joint angles (for smooth animation) ---
    ThetaOld = getappdata(0,'ThetaOld');
    if isempty(ThetaOld)
        ThetaOld = [0 0 0 0 0 0];
        setappdata(0,'ThetaOld',ThetaOld);
    end

    % --- Trajectory interpolation ---
    t1 = linspace(ThetaOld(1),theta1,n);
    t2 = linspace(ThetaOld(2),theta2,n);
    t3 = linspace(ThetaOld(3),theta3,n);
    t4 = linspace(ThetaOld(4),theta4,n);
    t5 = linspace(ThetaOld(5),theta5,n);
    t6 = linspace(ThetaOld(6),theta6,n);

    % --- Load link data ---
    load('linksdata.mat');

% --- Initialize patches if not already done or invalid ---
handles = getappdata(0,'patch_h');
if isempty(handles) || any(~isvalid(handles))  % ←★追加
    figure(1); clf; hold on; axis equal; view(3);
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('PUMA Robot Animation');

    L1 = patch('Vertices', s1.V1(:,1:3), 'Faces', s1.F1, 'FaceColor', [0.717,0.116,0.123]);
    L2 = patch('Vertices', s2.V2(:,1:3), 'Faces', s2.F2, 'FaceColor', [0.216,1,.583]);
    L3 = patch('Vertices', s3.V3(:,1:3), 'Faces', s3.F3, 'FaceColor', [0.306,0.733,1]);
    L4 = patch('Vertices', s4.V4(:,1:3), 'Faces', s4.F4, 'FaceColor', [1,0.542,0.493]);
    L5 = patch('Vertices', s5.V5(:,1:3), 'Faces', s5.F5, 'FaceColor', [0.216,1,.583]);
    L6 = patch('Vertices', s6.V6(:,1:3), 'Faces', s6.F6, 'FaceColor', [1,1,0.255]);
    L7 = patch('Vertices', s7.V7(:,1:3), 'Faces', s7.F7, 'FaceColor', [0.306,0.733,1]);
    Tr = plot3(0,0,0,'r','LineWidth',2);

    handles = [L1 L2 L3 L4 L5 L6 L7 Tr];

    setappdata(0,'patch_h',handles);
    setappdata(0,'xtrail',[]);
    setappdata(0,'ytrail',[]);
    setappdata(0,'ztrail',[]);
else
    L1 = handles(1);
    L2 = handles(2);
    L3 = handles(3);
    L4 = handles(4);
    L5 = handles(5);
    L6 = handles(6);
    L7 = handles(7);
    Tr = handles(8);
end


    % --- Animation loop ---
    for i = 2:n
        % Forward kinematics
        T_01 = tmat(0, 0, 0, t1(i));
        T_12 = tmat(-90, 0, 0, t2(i));
        T_23 = tmat(0, a2, d3, t3(i));
        T_34 = tmat(-90, a3, d4, t4(i));
        T_45 = tmat(90, 0, 0, t5(i));
        T_56 = tmat(-90, 0, 0, t6(i));

        T_02 = T_01*T_12;
        T_03 = T_02*T_23;
        T_04 = T_03*T_34;
        T_05 = T_04*T_45;
        T_06 = T_05*T_56;

        % Apply transformations to vertices
        Link1 = s1.V1;
        Link2 = (T_01*s2.V2')';
        Link3 = (T_02*s3.V3')';
        Link4 = (T_03*s4.V4')';
        Link5 = (T_04*s5.V5')';
        Link6 = (T_05*s6.V6')';
        Link7 = (T_06*s7.V7')';

        % Update patches
        set(L1,'Vertices',Link1(:,1:3));
        set(L2,'Vertices',Link2(:,1:3));
        set(L3,'Vertices',Link3(:,1:3));
        set(L4,'Vertices',Link4(:,1:3));
        set(L5,'Vertices',Link5(:,1:3));
        set(L6,'Vertices',Link6(:,1:3));
        set(L7,'Vertices',Link7(:,1:3));

        % --- Trail (軌跡) 更新 ---
        if trail == 'y'
            x_trail = getappdata(0,'xtrail');
            y_trail = getappdata(0,'ytrail');
            z_trail = getappdata(0,'ztrail');

            xdata = [x_trail T_06(1,4)];
            ydata = [y_trail T_06(2,4)];
            zdata = [z_trail T_06(3,4)];

            setappdata(0,'xtrail',xdata);
            setappdata(0,'ytrail',ydata);
            setappdata(0,'ztrail',zdata);

            set(Tr,'XData',xdata,'YData',ydata,'ZData',zdata);
        end

        drawnow;
    end

    % --- Store new theta for next motion ---
    setappdata(0,'ThetaOld',[theta1 theta2 theta3 theta4 theta5 theta6]);

    % --- End-effector position ---
    pos = T_06*[0 0 0 1]';
    px = pos(1); py = pos(2); pz = pos(3);
end

% --- Transformation Matrix Function ---
function T = tmat(alpha, a, d, theta)
    alpha = alpha*pi/180;
    theta = theta*pi/180;
    c = cos(theta); s = sin(theta);
    ca = cos(alpha); sa = sin(alpha);
    T = [c -s 0 a;
         s*ca c*ca -sa -sa*d;
         s*sa c*sa ca ca*d;
         0 0 0 1];
end
