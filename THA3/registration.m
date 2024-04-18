% 3d point set to 3d point set registration algorithm
function T_ab = registration(a,b)
    N = size(a,1);

    % step 1: computer a_bar and b_bar
    a_bar = sum(a,1)/N;
    b_bar = sum(b,1)/N;

    % step 2: a_tilde and b_tilde
    a_t = a - a_bar;
    b_t = b - b_bar;

    % step 3: find R using the SVD approach
    H = zeros(3,3);
    for i = 1:N
        H = H + a_t(i,:).' * b_t(i,:);
    end
    [U,~,V] = svd(H);
    R = V*U.';
    % % todo: verify det(R) = 1 and do something about it?

    % step 4: find p
    p = b_bar.' - R * a_bar.';

    % step 5: assemble transformation matrix
    T_ab = [R,p;0,0,0,1];
end
