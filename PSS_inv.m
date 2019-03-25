%����3-PSS-S�����˶�ѧ���
%rΪ��ƽ̨���Բ�뾶��LΪ��������֮��ı����˵ĳ��ȣ�
%hΪת�����ĵ���ƽ̨���ĵľ��룬HΪת�����ĵ��ƶ�������ľ���,thetaΪ�ƶ������ߺ�Z��Ľṹ��
%����ZYXŷ���ǣ�gammaΪZ��ת�ǣ�betaΪY��ת�ǣ�alphaΪX��ת��
function[R_t, d_c, A, B, a]=PSS_inv(r, L, h, H, gamma, beta, alpha, theta)
%������ת����
R11 = cos(gamma)*cos(beta);
R12 = cos(gamma)*sin(beta)*sin(alpha)-sin(gamma)*cos(alpha);
R13 = cos(gamma)*sin(beta)*cos(alpha)+sin(gamma)*sin(alpha);
R21 = sin(gamma)*cos(beta);
R22 = sin(gamma)*sin(beta)*sin(alpha)+cos(gamma)*cos(alpha);
R23 = sin(gamma)*sin(beta)*cos(alpha)-cos(gamma)*sin(alpha);
R31 = -sin(beta);
R32 = cos(beta)*sin(alpha);
R33 = cos(beta)*cos(alpha);

%������
%���㽻������H���ƶ�����S�µĵ�λ����
a1 = [sin(theta)*cos(-2*pi/3), -sin(theta)*sin(-2*pi/3), -cos(theta)];
a2 = [sin(theta)*cos(2*pi/3), -sin(theta)*sin(2*pi/3), -cos(theta)];
a3 = [sin(theta)*cos(0), -sin(theta)*sin(0), -cos(theta)];
%���㶯ƽ̨�ϵ�S���ڶ�����ϵ�µ�����
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

%�����ƶ����ĳ���
d1_1 = (-(V1-2*a1(3)*H)+sqrt((V1-2*a1(3)*H)^2-4*U1*(W1+H^2+2*H*g1(3))))/(2*U1);
%d1_2 = (-(V1-2*a1(3)*H)-sqrt((V1-2*a1(3)*H)^2-4*U1*(W1+H^2+2*H*g1(3))))/(2*U1);

d2_1 = (-(V2-2*a2(3)*H)+sqrt((V2-2*a2(3)*H)^2-4*U2*(W2+H^2+2*H*g2(3))))/(2*U2);
%d2_2 = (-(V2-2*a2(3)*H)-sqrt((V2-2*a2(3)*H)^2-4*U2*(W2+H^2+2*H*g2(3))))/(2*U2);

d3_1 = (-(V3-2*a3(3)*H)+sqrt((V3-2*a3(3)*H)^2-4*U3*(W3+H^2+2*H*g3(3))))/(2*U3);
%d3_2 = (-(V3-2*a3(3)*H)-sqrt((V3-2*a3(3)*H)^2-4*U3*(W3+H^2+2*H*g3(3))))/(2*U3);

d_c = [d1_1 d2_1 d3_1];
       %d1_2 d2_2 d3_2];
       
%�����ƶ����ϵ�S���ڶ�����ϵ�µ�����      
A1 = d_c(1)*a1-[0 0 H];
A2 = d_c(2)*a2-[0 0 H];
A3 = d_c(3)*a3-[0 0 H];

%���ؼ�����
R_t = [R11 R12 R13; R21 R22 R23; R31 R32 R33];
A = [A1;A2;A3];
B = [g1;g2;g3];
a = [a1;a2;a3];
end

