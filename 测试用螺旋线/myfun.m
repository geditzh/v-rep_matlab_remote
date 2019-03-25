function R_t = myfun(alpha,beta,gamma)
% R11 = cos(gamma)*cos(beta);
% R12 = cos(gamma)*sin(beta)*sin(alpha)-sin(gamma)*cos(alpha);
% R13 = cos(gamma)*sin(beta)*cos(alpha)+sin(gamma)*sin(alpha);
% R21 = sin(gamma)*cos(beta);
% R22 = sin(gamma)*sin(beta)*sin(alpha)+cos(gamma)*cos(alpha);
% R23 = sin(gamma)*sin(beta)*cos(alpha)-cos(gamma)*sin(alpha);
% R31 = -sin(beta);
% R32 = cos(beta)*sin(alpha);
% R33 = cos(beta)*cos(alpha);
% R_t = [R11 R12 R13; R21 R22 R23; R31 R32 R33];

rz = [cos(gamma) -sin(gamma) 0;sin(gamma) cos(gamma) 0;0 0 1];
ry = [cos(beta) 0 sin(beta);0 1 0;-sin(beta) 0 cos(beta)];
rx = [1 0 0;0 cos(alpha) -sin(alpha);0 sin(alpha) cos(alpha)];
R_t = rx*ry*rz;

end