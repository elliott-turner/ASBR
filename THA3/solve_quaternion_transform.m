% solve for unknown transfrom using quaternion analytical approach
function X = solve_quaternion_transform(q_robot,q_camera,t_robot,t_camera,n)
    % step 1: solve the rotation matrix R_x
    M = zeros(4*n,4);
    for i = 1:n
        q_a = q_robot(i,:).';
        q_b = q_camera(i,:).';
        s_a = q_a(1);
        v_a = q_a(2:4);
        s_b = q_b(1);
        v_b = q_b(2:4);
        M_n = [(s_a-s_b), -(v_a-v_b).';
            (v_a-v_b), (s_a-s_b)*eye(3) + sk(v_a+v_b)];
        M((i-1)*4+1:(i-1)*4+4,:) = M_n;
    end
    [~,~,V] = svd(M);
    q = V(4,:);
    R_x = quat2rotm(q);

    % step 2: solve for the translation vector p_x
    A = zeros(3*n,3);
    b = zeros(3*n,1);
    for i = 1:n
        q_a = q_robot(i,:);
        p_a = t_robot(i,:).';
        p_b = t_camera(i,:).';
        R_a = quat2rotm(q_a);
        A((i-1)*3+1:(i-1)*3+3,:) = R_a - eye(3);
        b((i-1)*3+1:(i-1)*3+3,:) = R_x * p_b - p_a;
    end
    p_x = A\b;
    X = [R_x, p_x; 0, 0, 0, 1];
end

% convert vector to skew symmetrtic matrix
function S = sk(v)
    S = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
end
