% returns test setup
function setup = test_setup()
    setup.Robot = loadrobot("frankaEmikaPanda");
    setup.LastJointIndex = 7;
end
