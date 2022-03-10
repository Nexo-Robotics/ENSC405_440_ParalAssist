
%% Design 1
L(1) = Link('revolute', 'd', 0.216,'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,'a', 0.8, 'alpha', 0, 'offset', pi/2);
L(3) = Link('revolute', 'd', 0,'a', sqrt(0.145^2+0.42746^2)+0.1, 'alpha',0, 'offset', -pi/2);

L(1).qlim = [-90, 90]/180*pi;
L(2).qlim = [-120, 30]/180*pi;
L(3).qlim = [-60, 100]/180*pi;

Four_dof = SerialLink(L,'name','4-dof');
Four_dof.base = transl(0,0,0.72);

num = 30000;
P = zeros(num, 3);
P1 = zeros(1, 3);
P2 = zeros(num, 3);

for i = 1:num
    q1 = L(1).qlim(1) + rand * (L(1).qlim(2) - L(1).qlim(1) );
    q2 = L(2).qlim(1) + rand * (L(2).qlim(2) - L(2).qlim(1) );
    q3 = L(3).qlim(1) + rand * (L(3).qlim(2) - L(3).qlim(1) );

    
    q = [q1 q2 q3];
    T = Four_dof.fkine(q);
    P(i, :) = transl(T);
    if (P(i, 1)<0 || P(i, 2)>0 || P(i, 3)<0.5)
        P1 = [P1 ; P(i, :)];
    else
        P2 = [P2 ; P(i, :)];
    end

end

plot3(P1(:,1),P1(:, 2),P1(:, 3), 'r.', 'markersize', 1);
hold on
plot3(P2(:,1),P2(:, 2),P2(:, 3), 'b.', 'markersize', 1);
hold off

Four_dof.teach


%% Backup1
L(1) = Link('revolute', 'd', 0.216,'a', 0, 'alpha', pi/2);
L(1).qlim = [-90, 90]/180*pi;

Four_dof = SerialLink(L,'name','4-dof');
Four_dof.base = transl(0,0,0.72);

num = 30000;
P = zeros(num, 3);
P1 = zeros(1, 3);
P2 = zeros(num, 3);

for i = 1:10000
    q1 = L(1).qlim(1) + rand * (L(1).qlim(2) - L(1).qlim(1) );
    q = [q1];
    T = Four_dof.fkine(q);
    P(i, :) = transl(T);
    if (P(i, 1)<0 || P(i, 2)>0 || P(i, 3)<0.5)
        P1 = [P1 ; P(i, :)];
    else
        P2 = [P2 ; P(i, :)];
    end

end

L(2) = Link('revolute', 'd', 0,'a', 0.8, 'alpha', 0, 'offset', pi/2);
L(2).qlim = [-120, 30]/180*pi;

Four_dof = SerialLink(L,'name','4-dof');
Four_dof.base = transl(0,0,0.72);

for i = 10001:20000
    q1 = L(1).qlim(1) + rand * (L(1).qlim(2) - L(1).qlim(1) );
    q2 = L(2).qlim(1) + rand * (L(2).qlim(2) - L(2).qlim(1) );
    q = [q1 q2];
    T = Four_dof.fkine(q);
    P(i, :) = transl(T);
    if (P(i, 1)<0 || P(i, 2)>0 || P(i, 3)<0.5)
        P1 = [P1 ; P(i, :)];
    else
        P2 = [P2 ; P(i, :)];
    end

end


L(3) = Link('revolute', 'd', 0,'a', 0.8, 'alpha',0, 'offset', -pi/2);
L(3).qlim = [-60, 100]/180*pi;

Four_dof = SerialLink(L,'name','4-dof');
Four_dof.base = transl(0,0,0.72);

for i = 20001:30000
    q1 = L(1).qlim(1) + rand * (L(1).qlim(2) - L(1).qlim(1) );
    q2 = L(2).qlim(1) + rand * (L(2).qlim(2) - L(2).qlim(1) );
    q3 = L(3).qlim(1) + rand * (L(3).qlim(2) - L(3).qlim(1) );
    q = [q1 q2 q3];
    T = Four_dof.fkine(q);
    P(i, :) = transl(T);
    if (P(i, 1)<0 || P(i, 2)>0 || P(i, 3)<0.5)
        P1 = [P1 ; P(i, :)];
    else
        P2 = [P2 ; P(i, :)];
    end

end

plot3(P1(:,1),P1(:, 2),P1(:, 3), 'r.', 'markersize', 1);
hold on
plot3(P2(:,1),P2(:, 2),P2(:, 3), 'b.', 'markersize', 1);
hold off

Four_dof.teach
%% Design 2
L(1) = Link('revolute', 'd', 0.216,'a', 0, 'alpha', pi/2);
L(2) = Link('revolute', 'd', 0,'a', 0.8, 'alpha', 0, 'offset', pi/2);
L(3) = Link('revolute', 'd', 0,'a', sqrt(0.145^2+0.42746^2), 'alpha',0, 'offset', -atan(9427.46/145));
L(4) = Link('revolute', 'd', 0,'a', 0, 'alpha', pi/2, 'offset', atan(427.46/145));
L(5) = Link('revolute', 'd', 0.258,'a', 0, 'alpha', 0);

L(1).qlim = [-180, 180]/180*pi;
L(2).qlim = [-45, 30]/180*pi;
L(3).qlim = [30, 100]/180*pi;
L(4).qlim = [-90, 90]/180*pi;
L(5).qlim = [-90, 90]/180*pi;

Five_dof = SerialLink(L,'name','5-dof');
Five_dof.base = transl(0,0,0.28)*troty(pi/2);

% q0 = [0 0 0 0 0];
% v = [35 20];
% w = [-1 1 -1 1 0 2];
% 
% Four_dof.plot3d(q0,'tilesize',0.1,'workspace',w,'path','C:\Users\Lawrence\Desktop\SFU\SP22\ENSC405','nowrist','view',v)
% 
% light('Position', [1 1 1], 'color','w');

num = 30000;
P = zeros(num, 3);
for i = 1:num
    q1 = L(1).qlim(1) + rand * (L(1).qlim(2) - L(1).qlim(1) );
    q2 = L(2).qlim(1) + rand * (L(2).qlim(2) - L(2).qlim(1) );
    q3 = L(3).qlim(1) + rand * (L(3).qlim(2) - L(3).qlim(1) );
    q4 = L(4).qlim(1) + rand * (L(4).qlim(2) - L(4).qlim(1) );
    q5 = L(5).qlim(1) + rand * (L(5).qlim(2) - L(5).qlim(1) );
    
    q = [q1 q2 q3 q4 q5];
    T = Five_dof.fkine(q);
    P(i, :) = transl(T);
end

% if (
plot3(P(:,1),P(:, 2),P(:, 3), 'b.', 'markersize', 1);

Five_dof.teach