function interactive_6dof_robot()
    % Build robot with visuals
    robot = build6DOFRobotWithCylinders();

    % Create figure and axes
    f = figure('Name','6-DOF Arm Control','Position',[100 100 1000 600]);
    ax = axes('Parent', f, 'Position', [0.3 0.2 0.65 0.75]);

    % Initial configuration
    q = zeros(1,6);
    show(robot, q, 'Parent', ax, 'PreservePlot', false);
    view(135, 25);
    axis equal;

    % Create sliders and labels
    sliders = gobjects(1,6);
    for i = 1:6
        sliders(i) = uicontrol('Style','slider',...
            'Min', -180, 'Max', 180, 'Value', 0,...
            'Position', [20, 500 - (i-1)*60, 200, 20],...
            'Callback', @(src,~) updateRobot());
        
        uicontrol('Style','text',...
            'Position',[20, 520 - (i-1)*60, 200, 20],...
            'String', sprintf('Joint %d (deg)', i));
    end

    % Update robot pose when sliders move
    function updateRobot()
        for j = 1:6
            q(j) = deg2rad(sliders(j).Value);
        end
        show(robot, q, 'Parent', ax, 'PreservePlot', false);
        drawnow;
    end
end

function robot = build6DOFRobotWithCylinders()
    % DH table: [theta d a alpha] in SI units (m, deg)
    dhparams = [...
        0     0.00000   0.00000     0;
        0     0.00000   0.00000    -90;
        0     0.00000   0.25000     0;
        0     0.00000   0.08416     90;
        0     0.01100   0.23350   -90;
        0     0.01200   0.11100    90];

    robot = rigidBodyTree('DataFormat','row','MaxNumBodies',6);

    % Visuals: approximate link lengths and radii
    link_lengths = [0.1, 0.1, 0.25, 0.084, 0.233, 0.111];  % in meters
    link_radii = 0.02 * ones(1,6);  % 2 cm cylinders

    for i = 1:6
        body = rigidBody(sprintf('link%d', i));
        joint = rigidBodyJoint(sprintf('joint%d', i), 'revolute');

        d     = dhparams(i,2);
        a     = dhparams(i,3);
        alpha = deg2rad(dhparams(i,4));

        % DH transform
        tform = trvec2tform([0 0 d]) * ...
                rotm2tform(axang2rotm([1 0 0 alpha])) * ...
                trvec2tform([a 0 0]);

        setFixedTransform(joint, tform);
        body.Joint = joint;

        % Add a cylinder visual (aligned along Z, offset upward)
        cyl_length = link_lengths(i);
        cyl_radius = link_radii(i);
        cyl_offset = trvec2tform([0 0 cyl_length/2]);  % shift center to base
        addVisual(body, 'Cylinder', [cyl_radius, cyl_length], cyl_offset);

        % Attach to tree
        if i == 1
            addBody(robot, body, 'base');
        else
            addBody(robot, body, sprintf('link%d', i-1));
        end
    end
end
