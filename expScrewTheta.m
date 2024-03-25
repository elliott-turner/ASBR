% calculate exp(screw axis, theta)
function T = expScrewTheta(screwAxis, theta)
    omega = screwAxis(1:3);
    v = screwAxis(4:6);
    Omega = [0, -omega(3), omega(2);
            omega(3), 0, -omega(1);
            -omega(2), omega(1), 0];
    Omega = [Omega, v; 0 0 0 0];
    T = expm(Omega * theta);
end