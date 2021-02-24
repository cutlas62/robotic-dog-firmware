a = 50;     % Length of first segment
b = 50;     % Length of second segment
delay = 0.005;
clf

for x = 50:-1:-30
    y = -80;
    [q1, q2] = calcJointAngles (x, y, a, b);
    plotLeg (q1, q2, a, b);
    pause(delay)
end

for y = -80:1:-60
    x = -30;
    [q1, q2] = calcJointAngles (x, y, a, b);
    plotLeg (q1, q2, a, b);
    pause(delay)
end

for x = -30:1:50
    y = -60;
    [q1, q2] = calcJointAngles (x, y, a, b);
    plotLeg (q1, q2, a, b);
    pause(delay)
end

for y = -60:-1:-80
    x = 50;
    [q1, q2] = calcJointAngles (x, y, a, b);
    plotLeg (q1, q2, a, b);
    pause(delay)
end

% Local functions
function [q1, q2] = calcJointAngles(x, y, a, b)
    % Calculate the joint angles (q1,q2) from the global coordinates
    % of the foot (x,y) and the length of the limbs
    q2 = acos((x*x + y*y - a*a - b*b)/(2*a*b));
    q1 = atan2(y,x) - atan2((b*sin(q2)),(a+b*cos(q2)));
end

function plotLeg(q1, q2, a, b)
    % Plot the leg given the length of the segments and the joint angles
    plot([0 a*cos(q1)], [0 a*sin(q1)], '-o')
    axis([-100 100 -100 100])
    pbaspect([1 1 1])
    hold on
    plot([a*cos(q1) a*cos(q1)+b*cos(q1+q2)], [a*sin(q1) a*sin(q1)+b*sin(q1+q2)], '-o')
    hold off
end