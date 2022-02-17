%{
    Copyright (c) 2022 Yosep Kim.
    See LICENSE for more information
    https://github.com/ypskm/iklib
%}

syms theta1 theta2 theta3 theta4 theta5 theta6
syms alpha1 alpha2 alpha3 alpha4 alpha5 alpha6
syms a1 a2 a3 a4 a5 a6
syms d1 d2 d3 d4 d5 d6 dg
syms roll pitch yaw
syms mkdh_roll mkdh_pitch mkdh_yaw

T = @(theta_cur, alpha_prev, a_prev, d_cur) ([cos( theta_cur ), -sin( theta_cur ), 0, a_prev;sin( theta_cur) * cos( alpha_prev), cos( theta_cur) * cos( alpha_prev ), -sin( alpha_prev) , - d_cur * sin( alpha_prev); sin( theta_cur ) * sin( alpha_prev ), cos( theta_cur ) * sin( alpha_prev) , cos( alpha_prev ),  d_cur * cos( alpha_prev);0, 0, 0, 1]);
Rx = @(angle)([1., 0., 0.;0., cos(angle),-sin(angle);0., sin(angle), cos(angle)]);
Ry = @(angle)([ cos(angle), 0., sin(angle); 0., 1., 0.;-sin(angle), 0., cos(angle)]);
Rz = @(angle)([cos(angle),-sin(angle), 0.;sin(angle), cos(angle), 0.;0., 0., 1.]);

% R= X(roll)*Y(pitch)*Z(yaw)
% [0/G], -roll, -pitch, -yaw, -mkdh_roll, -mkdh_pitch, -mkdh_yaw 
% inv(Rmkdh * Rwrist)   == inv(Rwrist) * inv(Rmkdh)
%                       == inv(Rx(-roll)*Ry(-pitch)*Rz(-yaw)) *inv(Rx(-mkdh_roll)*Ry(-mkdh_pitch)*Rz(-mkdh_yaw))
%                       == inv(Rz(-yaw))*inv(Ry(-pitch))*inv(Rx(-roll))*inv(Rz(-mkdh_yaw))*inv(Ry(-mkdh_pitch))*inv(Rx(-mkdh_roll))
%                       == Rz(yaw)) *Ry(pitch) * Rx(roll) * Rz(mkdh_yaw) * Ry(mkdh_pitch) * Rx(mkdh_roll)
R_0G_NONDH = Rz(yaw) * Ry(pitch) * Rx(roll)  ;
R_MKDH = Rz(mkdh_yaw) * Ry(mkdh_pitch) * Rx(mkdh_roll);
R0G = R_0G_NONDH * R_MKDH; 

% R0G   == R03 * R36 * R6G
%       == R03 * R36 * eye(3)
% transpose(R03) * R0G == trnaspose(R03) * R03 * R36
%                      == eye(3) * R36
% thus, R36 = transpose(R03) * R0G
T01 = T(theta1, 0, 0, d1);
T12 = T(theta2, alpha1, a1, d2);
T23 = T(theta3, alpha2, a2, d3);
T03 = T01*T12*T23;
R03 = T03(1:3, 1:3);
R36 = transpose(R03) * R0G;

% another way to get R36
% T34 = T(theta4, alpha3, a3, d4);
% T45 = T(theta5, alpha4, a4, d5);
% T56 = T(theta6, alpha5, a5, d6);
% T6g = T(0, 0, 0, dg);
% T36 = T34*T45*T56;
% R36 = T36(1:3, 1:3);

for i = 1:3
    for j=1:3
        fprintf("void get_r0g_%d%d(ik_state_t*ik_state) {ik_state->r0g[%d][%d] = %s;pthread_exit(NULL);}\n", i-1, j-1, i-1, j-1, R0G(i, j));
    end
end

for i = 1:3
    for j=1:3
        fprintf("void* get_r36_%d%d(ik_state_t*ik_state) {ik_state->r36[%d][%d] = %s;pthread_exit(NULL);}\n", i-1, j-1, i-1, j-1, R36(i, j));
    end
end