function setup = test_setup_pa4()
    % load robot
    robot = loadrobot('frankaEmikaPanda');
    lastJointIndex = 8;

    % remove stock gripper
    numBodies = numel(robot.Bodies);
    for i = numBodies:-1:lastJointIndex
        removeBody(robot,robot.Bodies{i}.Name);
    end

    % add cylindrical tool
    tool_name = 'cylindrical_tool';
    tool_R = [1,0,0;0,0,-1;0,1,0];
    tool_p = [0;0;-0.1];
    tool_body = rigidBody(tool_name);
    addVisual(tool_body,'mesh','tool.stl',[tool_R,tool_p;0,0,0,1]);
    addBody(robot,tool_body,robot.Bodies{lastJointIndex-1}.Name);
    setFixedTransform(robot.Bodies{lastJointIndex}.Joint,trvec2tform([0,0,0.21]));

    % save modified robotRBT and last joint index to returned struct
    setup.Robot = robot;
    setup.LastJointIndex = lastJointIndex;
