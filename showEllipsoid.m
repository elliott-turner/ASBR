% plot 3D ellipsoid centered at point p based on eigenvalues/vectors of A
function showEllipsoid(p, A, scaleFactor)
    [v, d] = eig(A);

    v1 = v(1:3, 1) .* (d(1, 1) * scaleFactor);
    v2 = v(1:3, 2) .* (d(2, 2) * scaleFactor);
    v3 = v(1:3, 3) .* (d(3, 3) * scaleFactor);

    [x, y, z] = ellipsoid(0, 0, 0, 1, 1, 1);
    P = p(:) + v1(:)*x(:)' + v2(:)*y(:)' + v3(:)*z(:)';

    X = reshape(P(1,:), size(x));
    Y = reshape(P(2,:), size(y));
    Z = reshape(P(3,:), size(z));
    surf(X, Y, Z, 'FaceColor', 'yellow', 'EdgeColor', 'none', 'FaceAlpha', 0.5);
end
