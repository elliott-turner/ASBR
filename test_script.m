% this is just a test script

% load robot rigid body tree
robot = loadrobot("frankaEmikaPanda");

% joint indices beyond this will not be included in calculations
lastJointIndex = 7;

% % define specific config
% theta = [0, 0, 0, 0, 0, 0, 0, 0, 0];
% config = thetaToConfiguration(theta, robot);

% get random configuration
config = randomConfiguration(robot);

% calculate forward kinematics using space form of exp. prods.
FK_space(robot, config, lastJointIndex)

% verify result
getTransform(robot, config, robot.BodyNames{lastJointIndex})
