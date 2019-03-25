%计算3-PSS-S机构运动学逆解
%r为动平台外接圆半径，L为两个铰链之间的被动杆的长度，
%h为转动中心到动平台中心的距离，H为转动中心到移动副交点的距离,theta为移动副轴线和Z轴的结构角
%采用ZYX欧拉角，gamma为Z轴转角，beta为Y轴转角，alpha为X轴转角
function[R_t, d_c, A, B, a]=PSS_inv(r, L, h, H, gamma, beta, alpha, theta)
%计算旋转矩阵
R11 = cos(gamma)*cos(beta);
R12 = cos(gamma)*sin(beta)*sin(alpha)-sin(gamma)*cos(alpha);
R13 = cos(gamma)*sin(beta)*cos(alpha)+sin(gamma)*sin(alpha);
R21 = sin(gamma)*cos(beta);
R22 = sin(gamma)*sin(beta)*sin(alpha)+cos(gamma)*cos(alpha);
R23 = sin(gamma)*sin(beta)*cos(alpha)-cos(gamma)*sin(alpha);
R31 = -sin(beta);
R32 = cos(beta)*sin(alpha);
R33 = cos(beta)*cos(alpha);

%逆解计算
%计算交点中心H到移动副上S铰的单位向量
a1 = [sin(theta)*cos(-2*pi/3), -sin(theta)*sin(-2*pi/3), -cos(theta)];
a2 = [sin(theta)*cos(2*pi/3), -sin(theta)*sin(2*pi/3), -cos(theta)];
a3 = [sin(theta)*cos(0), -sin(theta)*sin(0), -cos(theta)];
%计算动平台上的S铰在定坐标系下的坐标
% g1 = r*[R11*cos(-pi)-R12*sin(-pi), R21*cos(-pi)-R22*sin(-pi), R31*cos(-pi)-R32*sin(-pi)+h/r];
% g2 = r*[R11*cos(pi/3)-R12*sin(pi/3), R21*cos(pi/3)-R22*sin(pi/3), R31*cos(pi/3)-R32*sin(pi/3)+h/r];
% g3 = r*[R11*cos(-pi/3)-R12*sin(-pi/3), R21*cos(-pi/3)-R22*sin(-pi/3), R31*cos(-pi/3)-R32*sin(-pi/3)+h/r];
g1 = r*[R11*cos(-pi)-R12*sin(-pi)+R13*h/r, R21*cos(-pi)-R22*sin(-pi)+R23*h/r, R31*cos(-pi)-R32*sin(-pi)+R33*h/r];
g2 = r*[R11*cos(pi/3)-R12*sin(pi/3)+R13*h/r, R21*cos(pi/3)-R22*sin(pi/3)+R23*h/r, R31*cos(pi/3)-R32*sin(pi/3)+R33*h/r];
g3 = r*[R11*cos(-pi/3)-R12*sin(-pi/3)+R13*h/r, R21*cos(-pi/3)-R22*sin(-pi/3)+R23*h/r, R31*cos(-pi/3)-R32*sin(-pi/3)+R33*h/r];

U1 = a1(1)^2 + a1(2)^2 + a1(3)^2;
U2 = a2(1)^2 + a2(2)^2 + a2(3)^2;
U3 = a3(1)^2 + a3(2)^2 + a3(3)^2;

V1 = -2*(a1(1)*g1(1)+a1(2)*g1(2)+a1(3)*g1(3));
V2 = -2*(a2(1)*g2(1)+a2(2)*g2(2)+a2(3)*g2(3));
V3 = -2*(a3(1)*g3(1)+a3(2)*g3(2)+a3(3)*g3(3));

W1 = g1(1)^2+g1(2)^2+g1(3)^2-L^2;
W2 = g2(1)^2+g2(2)^2+g2(3)^2-L^2;
W3 = g3(1)^2+g3(2)^2+g3(3)^2-L^2;

%计算移动副的长度
d1_1 = (-(V1-2*a1(3)*H)+sqrt((V1-2*a1(3)*H)^2-4*U1*(W1+H^2+2*H*g1(3))))/(2*U1);
%d1_2 = (-(V1-2*a1(3)*H)-sqrt((V1-2*a1(3)*H)^2-4*U1*(W1+H^2+2*H*g1(3))))/(2*U1);

d2_1 = (-(V2-2*a2(3)*H)+sqrt((V2-2*a2(3)*H)^2-4*U2*(W2+H^2+2*H*g2(3))))/(2*U2);
%d2_2 = (-(V2-2*a2(3)*H)-sqrt((V2-2*a2(3)*H)^2-4*U2*(W2+H^2+2*H*g2(3))))/(2*U2);

d3_1 = (-(V3-2*a3(3)*H)+sqrt((V3-2*a3(3)*H)^2-4*U3*(W3+H^2+2*H*g3(3))))/(2*U3);
%d3_2 = (-(V3-2*a3(3)*H)-sqrt((V3-2*a3(3)*H)^2-4*U3*(W3+H^2+2*H*g3(3))))/(2*U3);

d_c = [d1_1 d2_1 d3_1];
       %d1_2 d2_2 d3_2];
       
%计算移动副上的S铰在定坐标系下的坐标      
A1 = d_c(1)*a1-[0 0 H];
A2 = d_c(2)*a2-[0 0 H];
A3 = d_c(3)*a3-[0 0 H];

%返回计算结果
R_t = [R11 R12 R13; R21 R22 R23; R31 R32 R33];
A = [A1;A2;A3];
B = [g1;g2;g3];
a = [a1;a2;a3];
end

