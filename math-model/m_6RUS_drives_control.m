function output_values = m_6RUS_drives_control(...
    x, y, z, ...
    alpha, phi, theta, ...
    robot_constants)

N = robot_constants.numberOfLegs;
output_values = zeros(1, N);

legEnds = calculate_leg_ends_coordinates(x, y, z, alpha, phi, theta, ...
    robot_constants);

for i = 1:N
    ex = legEnds(1, i);
    ey = legEnds(2, i);
    ez = legEnds(3, i);
    
    solution = solve_inverse_kinematics(ex, ey, ez, robot_constants);
    output_values(i) = solution(1);
end

end



function solution = solve_inverse_kinematics(ex, ey, ez, robot_constants)

sign1 = +1;
sign2 = -1;

lRU = robot_constants.legRUChainLength;
lUS = robot_constants.legUSChainLength;

angUh = atan2(sign1*sign2 * (2*ex*lRU), ...
    sign1*sign2 * (lRU^2 + lUS^2 - (ex^2 + ey^2 + ez^2)));

radixExpr = ex^4 + 2*ex^2 * (lRU^2 - lUS^2 + (ey^2 + ez^2)) + ...
    (lRU^2 + lUS^2 - (ey^2 + ez^2))^2;

angUv = atan2(-sign1 * sqrt(4*lRU^2*lUS^2 - radixExpr), ...
    -sign1*sign2 * sqrt(radixExpr));

AExpr = lRU + lUS*cos(angUh)*cos(angUv);
BExpr = lUS*sin(angUv);

angR = atan2(-AExpr*ey - BExpr*ez, -BExpr*ey + AExpr*ez);

solution = [angR, angUh, angUv];

end



function legEnds = calculate_leg_ends_coordinates(...
    x, y, z, ...
    alpha, phi, theta, ...
    robot_constants)

N = robot_constants.numberOfLegs;

R_body = robot_constants.bodyRadius;
legAttachmentDisplacement = [R_body, 0, 0] + ...
    robot_constants.legAttachmentDisplacement';

R_endEff = robot_constants.endEffectorRadius;
sphericalJointDisplacement = [R_endEff, 0, 0] + ...
    robot_constants.sphericalJointDisplacement';

asymm = robot_constants.jointSPositionAsymmetry;
sphericalJointDisplacement = sphericalJointDisplacement + ...
    [zeros(N, 1), (R_endEff/sqrt(3))*asymm * (-1).^(0:N-1)', zeros(N, 1)];

ang = (0:N-1)' * (2*pi/N);
leg_CS_orientation_quats = [cos(ang/2), ...
    sin(ang/2) .* repmat([0, 0, 1], N, 1)];
leg_CS_orientation_quats_inv = inverse_quat(leg_CS_orientation_quats);

end_effector_displacement = [x, y, z];
end_effector_rotation_quat = ...
    [cos(alpha/2), sin(alpha/2) * ...
    [cos(theta) * [cos(phi), sin(phi)], sin(theta)]];

end_effector_points_in_end_effector_CS = ...
    rotate_vec(leg_CS_orientation_quats_inv, sphericalJointDisplacement);

end_effector_points = ...
    rotate_vec(end_effector_rotation_quat, ...
    end_effector_points_in_end_effector_CS) + end_effector_displacement;

legEnds = rotate_vec(leg_CS_orientation_quats, end_effector_points) - ...
    legAttachmentDisplacement;
legEnds = legEnds';

end



function rv = rotate_vec(q, v)

q = [q(:, 1), -q(:, 2:4)];
rv = quatrotate(q, v);

end



function iq = inverse_quat(q)

iq = quatinv(q);

end
