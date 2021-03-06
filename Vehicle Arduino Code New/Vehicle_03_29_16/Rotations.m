syms yaw heading pitch p x y z

R1 = [[1, 0, 0]; ...
      [0, -1, 0];...
      [0 0 -1]];
    
RYaw = [[cos(heading + yaw), -sin(heading + yaw), 0]; ...
        [sin(heading + yaw), cos(heading + yaw), 0];...
        [0 0 1]];
    
RCam = [[0, 0, 1]; ...
        [1, 0, 0];...
        [0 1 0]];
                  
% RPitch = [[cos(pitch), 0, sin(pitch)];...
%           [0, 1, 0];...
%           [-sin(pitch), 0, cos(pitch)]];
      
RPitch = [[1 0 0];...
          [0, cos(pitch), -sin(pitch)];...
          [0, sin(pitch),  cos(pitch)]];

p = [x, y, z]';
assume(x,'real')
assume(y,'real')
assume(z,'real')
assume(p,'real')

R =  R1*RYaw*RCam*RPitch; 

assume(x,'real')
assume(y,'real')
assume(z,'real')
assume(p,'real')
p_prime = simplify(R*p)

RYaw = [[cos(-heading), -sin(-heading), 0]; ...
        [sin(-heading), cos(-heading), 0];...
        [0 0 1]];

    R2 = R1*RYaw
    p_prime2 = simplify(R2*p)
    
pitch = 0;
yaw = 0;
heading = 0;

close all
plot3([1 0], [0 0], [0 0], 'r-');
hold on
axis equal
axis vis3d
grid on

plot3([0 0], [1 0], [0 0], 'g-');
plot3([0 0], [0 0], [1 0], 'b-');

x = 2;
y = 0;
z = 5;



p_prime = simplify(subs(p_prime));
plot3(p_prime(1), p_prime(2), p_prime(3), 'xk')

