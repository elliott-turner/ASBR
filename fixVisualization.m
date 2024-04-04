% idk why but this seems to fix stuff
function fixVisualization()
    axis equal; % makes axes proportional
    xlim([-1.2, 1.2]);
    ylim([-1.2, 1.2]);
    zlim([-0.5, 1.8]);
    view(3); % force 3D view (fixes "stretching" issues?)
    camlight; % make shaded
    %set(findall(gca, 'Type', 'patch'), 'FaceAlpha', 0.5); % make transparent
    axis vis3d; % keeps it from zooming in and out to fit in plot window
    grid on; % background grid
end
