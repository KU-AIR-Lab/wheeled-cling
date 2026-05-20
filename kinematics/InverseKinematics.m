clear
clc

% =======================================================================
% ROBOT PARAMETERS

R = 389e-3; % robot circumradius
d = 100e-3; % wheel diameter

% Transformation from robot frame to pyramid-segment-fixed frames

R1 = [cosd(90)   -sind(90);
      sind(90)    cosd(90)];

R2 = [cosd(-30)  -sind(-30);
      sind(-30)   cosd(-30)];

R3 = [cosd(210)  -sind(210);
      sind(210)   cosd(210)];

% =======================================================================
% MOTION COMMANDS
% Uncomment to apply

% % Robot moving forward
% Vx = 0;
% Vy = 0.03;
% Omega = 0;

% % Robot moving backward
% Vx = 0;
% Vy = -0.03;
% Omega = 0;

% % Robot moving right
% Vx = 0.03;
% Vy = 0;
% Omega = 0;

% % Robot moving left
% Vx = -0.03;
% Vy = 0;
% Omega = 0;

% % Robot moving diagonally 45 degrees
% Vx = 0.03;
% Vy = 0.03;
% Omega = 0;

% % Pure rotation
Vx = 0;
Vy = 0;
Omega = 0.1;

% =======================================================================
% DERIVED VARIABLES

Yaw = atan2(Vy,Vx);
V_mag = sqrt(Vx^2 + Vy^2);

V   = [Vx; Vy];
Vti = [0; Omega*R];

Vt1 = R1*Vti;
Vt2 = R2*Vti;
Vt3 = R3*Vti;

tol = 1e-12;

% =======================================================================
% INVERSE KINEMATICS

% Wheel-frame orientations in robot frame
alpha1 = pi/2;
alpha2 = -30*pi/180;
alpha3 = 210*pi/180;

% -----------------------------------------------------------------------
% Wheel 1

V1 = Vt1 + V;

V1x = V1(1);
V1y = V1(2);

Theta1_raw = atan2(V1y,V1x);
V1_mag = sqrt(V1x^2 + V1y^2);

Omega1 = V1_mag/(d/2);

angle1_raw = wrapToPi(Theta1_raw - alpha1);

[angle1, Omega1] = limitSteeringAngle(angle1_raw, Omega1);

Theta1 = alpha1 + angle1;

% -----------------------------------------------------------------------
% Wheel 2

V2 = Vt2 + V;

V2x = V2(1);
V2y = V2(2);

Theta2_raw = atan2(V2y,V2x);
V2_mag = sqrt(V2x^2 + V2y^2);

Omega2 = V2_mag/(d/2);

angle2_raw = wrapToPi(Theta2_raw - alpha2);

[angle2, Omega2] = limitSteeringAngle(angle2_raw, Omega2);

Theta2 = alpha2 + angle2;

% -----------------------------------------------------------------------
% Wheel 3

V3 = Vt3 + V;

V3x = V3(1);
V3y = V3(2);

Theta3_raw = atan2(V3y,V3x);
V3_mag = sqrt(V3x^2 + V3y^2);

Omega3 = V3_mag/(d/2);

angle3_raw = wrapToPi(Theta3_raw - alpha3);

[angle3, Omega3] = limitSteeringAngle(angle3_raw, Omega3);

Theta3 = alpha3 + angle3;

% =======================================================================
% DRAW ROBOT

P_top   = [0, R + R*sind(30)];
P_right = [ R*cosd(30), 0];
P_left  = [-R*cosd(30), 0];

O = [0, R*sind(30)];

W1 = P_top;
W2 = P_right;
W3 = P_left;

figure;
hold on;
grid on;
axis equal;

% -----------------------------------------------------------------------
% Triangle

tri_x = [P_top(1), P_right(1), P_left(1), P_top(1)];
tri_y = [P_top(2), P_right(2), P_left(2), P_top(2)];

plot(tri_x, tri_y, 'k-', 'LineWidth', 1.5);

% -----------------------------------------------------------------------
% Center

plot(O(1), O(2), 'ro', 'MarkerFaceColor', 'r');

text(O(1)-0.03, O(2)+0.02, 'O', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w');

% -----------------------------------------------------------------------
% Vertices

A = P_top;
B = P_right;
C = P_left;

text(A(1)-0.03, A(2)+0.025, 'A', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w');

text(B(1)+0.025, B(2)-0.035, 'B', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w');

text(C(1)-0.04, C(2)-0.035, 'C', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w');

% -----------------------------------------------------------------------
% Medians

M_A = (B + C)/2;
M_B = (A + C)/2;
M_C = (A + B)/2;

plot([A(1), M_A(1)], [A(2), M_A(2)], 'k--');
plot([B(1), M_B(1)], [B(2), M_B(2)], 'k--');
plot([C(1), M_C(1)], [C(2), M_C(2)], 'k--');

% -----------------------------------------------------------------------
% Robot Frame

frameLen = 0.10;

quiver(O(1), O(2), frameLen, 0, 0, ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 1.2);

quiver(O(1), O(2), 0, frameLen, 0, ...
    'm', 'LineWidth', 2, 'MaxHeadSize', 1.2);

text(O(1)+frameLen+0.01, O(2)-0.01, ...
    'X^+', ...
    'Color', 'm', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w');

text(O(1)-0.015, O(2)+frameLen+0.01, ...
    'Y^+', ...
    'Color', 'm', ...
    'FontWeight', 'bold', ...
    'BackgroundColor', 'w');

% -----------------------------------------------------------------------
% Local Frames

localLen = 0.08;

drawLocalFrame(A, O, localLen, '1');
drawLocalFrame(B, O, localLen, '2');
drawLocalFrame(C, O, localLen, '3');

% -----------------------------------------------------------------------
% Wheels

drawWheel(W1, d, Theta1, 'b');
drawWheel(W2, d, Theta2, 'b');
drawWheel(W3, d, Theta3, 'b');

plot(W1(1), W1(2), 'ko', 'MarkerFaceColor', 'k');
plot(W2(1), W2(2), 'ko', 'MarkerFaceColor', 'k');
plot(W3(1), W3(2), 'ko', 'MarkerFaceColor', 'k');

% -----------------------------------------------------------------------
% Wheel Directions

dir1 = getDirection(Omega1);
dir2 = getDirection(Omega2);
dir3 = getDirection(Omega3);

% -----------------------------------------------------------------------
% Wheel Labels

text(W1(1)+0.02, W1(2)+0.04, ...
    sprintf('Wheel 1 (%s)\n\\theta_1 = %.1f^\\circ', ...
    dir1, rad2deg(angle1)), ...
    'FontSize', 9, ...
    'BackgroundColor', 'w', ...
    'Margin', 2);

text(W2(1)+0.08, W2(2)+0.03, ...
    sprintf('Wheel 2 (%s)\n\\theta_2 = %.1f^\\circ', ...
    dir2, rad2deg(angle2)), ...
    'FontSize', 9, ...
    'BackgroundColor', 'w', ...
    'Margin', 2);

text(W3(1)-0.20, W3(2)+0.03, ...
    sprintf('Wheel 3 (%s)\n\\theta_3 = %.1f^\\circ', ...
    dir3, rad2deg(angle3)), ...
    'FontSize', 9, ...
    'BackgroundColor', 'w', ...
    'Margin', 2);

% -----------------------------------------------------------------------
% Information Text

infoText1 = sprintf(['V = %.3f m/s\n' ...
                     'Yaw = %.2f deg\n' ...
                     'Omega = %.3f rad/s'], ...
                     V_mag, rad2deg(Yaw), Omega);

text(-0.48, 0.66, infoText1, ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', 'k');

infoText2 = sprintf(['Steering Angle 1 = %.2f deg\n' ...
                     'Steering Angle 2 = %.2f deg\n' ...
                     'Steering Angle 3 = %.2f deg'], ...
                     rad2deg(angle1), ...
                     rad2deg(angle2), ...
                     rad2deg(angle3));

text(-0.48, 0.50, infoText2, ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', 'k');

infoText3 = sprintf(['Wheel Speed 1 = %.2f rad/s\n' ...
                     'Wheel Speed 2 = %.2f rad/s\n' ...
                     'Wheel Speed 3 = %.2f rad/s'], ...
                     Omega1, Omega2, Omega3);

text(-0.48, 0.34, infoText3, ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', 'w', ...
    'EdgeColor', 'k');

% -----------------------------------------------------------------------
% Motion Visualization

if abs(Omega) < tol && abs(V_mag) > tol

    arrowLength = 0.18;

    ux = cos(Yaw);
    uy = sin(Yaw);

    quiver(O(1), O(2), ...
        arrowLength*ux, ...
        arrowLength*uy, ...
        0, ...
        'r', ...
        'LineWidth', 2, ...
        'MaxHeadSize', 1.5);

elseif abs(V_mag) < tol && abs(Omega) > tol

    arcRadius = 0.13;

    if Omega > 0
        drawCircularArrow(O, arcRadius, 'CCW', 'r');
    else
        drawCircularArrow(O, arcRadius, 'CW', 'r');
    end
end

% -----------------------------------------------------------------------

xlabel('X [m]');
ylabel('Y [m]');

title('Top View of 3-Wheeled Triangle Robot');

axis([-0.5 0.5 -0.1 0.7]);

% =======================================================================
% FUNCTIONS
% =======================================================================

function drawWheel(center,d,angle,color)

    r = d/2;

    dx = r*cos(angle);
    dy = r*sin(angle);

    x = [center(1)-dx, center(1)+dx];
    y = [center(2)-dy, center(2)+dy];

    plot(x, y, color, 'LineWidth', 4);

end

% =======================================================================

function dir = getDirection(omega)

    if omega > 0
        dir = 'CCW';
    elseif omega < 0
        dir = 'CW';
    else
        dir = 'STOP';
    end

end

% =======================================================================

function drawCircularArrow(center, radius, direction, color)

    if strcmp(direction, 'CCW')
        t = linspace(-40,260,100)*pi/180;
    else
        t = linspace(260,-40,100)*pi/180;
    end

    x = center(1) + radius*cos(t);
    y = center(2) + radius*sin(t);

    plot(x, y, color, 'LineWidth', 2);

    xEnd = x(end);
    yEnd = y(end);

    xPrev = x(end-3);
    yPrev = y(end-3);

    dx = xEnd - xPrev;
    dy = yEnd - yPrev;

    quiver(xPrev, yPrev, 3*dx, 3*dy, ...
        0, color, ...
        'LineWidth', 2, ...
        'MaxHeadSize', 1.5);

end

% =======================================================================

function drawLocalFrame(P, O, len, labelNum)

    xAxis = (P - O);
    xAxis = xAxis/norm(xAxis);

    yAxis = [-xAxis(2), xAxis(1)];

    quiver(P(1), P(2), ...
        len*xAxis(1), ...
        len*xAxis(2), ...
        0, ...
        'm', ...
        'LineWidth', 1.8, ...
        'MaxHeadSize', 1.2);

    quiver(P(1), P(2), ...
        len*yAxis(1), ...
        len*yAxis(2), ...
        0, ...
        'm', ...
        'LineWidth', 1.8, ...
        'MaxHeadSize', 1.2);

    text(P(1)+1.40*len*xAxis(1), ...
         P(2)+1.40*len*xAxis(2), ...
         ['X_' labelNum '^+'], ...
         'Color', 'm', ...
         'FontWeight', 'bold', ...
         'BackgroundColor', 'w');

    text(P(1)+1.20*len*yAxis(1), ...
         P(2)+1.30*len*yAxis(2), ...
         ['Y_' labelNum '^+'], ...
         'Color', 'm', ...
         'FontWeight', 'bold', ...
         'BackgroundColor', 'w');

end

% =======================================================================

function [angleLimited, omegaSigned] = limitSteeringAngle(angleRaw, omegaRaw)

    angleLimited = wrapToPi(angleRaw);
    omegaSigned = omegaRaw;

    if angleLimited > pi/2

        angleLimited = angleLimited - pi;
        omegaSigned = -omegaSigned;

    elseif angleLimited < -pi/2

        angleLimited = angleLimited + pi;
        omegaSigned = -omegaSigned;

    end

end
