function T = scara_fkine_manual(theta1, theta2, q3, d1, d2, d3, d4)
%SCARA_FKINE_MANUAL Closed-form forward kinematics for the SCARA robot.

theta12 = theta1 + theta2;

c1 = cos(theta1);
s1 = sin(theta1);
c12 = cos(theta12);
s12 = sin(theta12);

px = d4 * c12 + d3 * c1;
py = d4 * s12 + d3 * s1;
pz = d1 + d2 - q3;

T = [c12,  s12, 0, px;
     s12, -c12, 0, py;
     0,      0, -1, pz;
     0,      0,  0, 1];
end
