% add reference frame origin to 3D plot
function showFrame(T, length)
    origin = T(1:3, end); % origin of frame in 3D space
    axes = T(1:3, 1:3) * eye(3); % correctly oriented axes
    axes = axes .* length; % scale axes to length
    % plot axes at origin
    quiver3(origin(1), origin(2), origin(3), axes(1,1), axes(2,1), axes(3,1), 'LineWidth', 4, 'Color', 'red');
    quiver3(origin(1), origin(2), origin(3), axes(1,2), axes(2,2), axes(3,2), 'LineWidth', 4, 'Color', 'green');
    quiver3(origin(1), origin(2), origin(3), axes(1,3), axes(2,3), axes(3,3), 'LineWidth', 4, 'Color', 'blue');
end
