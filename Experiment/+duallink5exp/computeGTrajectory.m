function result = computeGTrajectory(thetaDeg, phiDeg, geometry)
% Inputs are calibrated logical theta/phi angles in degrees. This adapter
% deliberately does not infer device channels, signs, zero offsets, or gearing.
thetaLogicalDeg = thetaDeg(:);
phiLogicalDeg = phiDeg(:);
if numel(thetaLogicalDeg) ~= numel(phiLogicalDeg)
    error('duallink5exp:SizeMismatch', ...
        'thetaDeg and phiDeg must have the same length.');
end

result.x = nan(size(thetaLogicalDeg));
result.y = nan(size(phiLogicalDeg));
result.status = strings(size(thetaLogicalDeg));
for index = 1:numel(thetaLogicalDeg)
    if ~isfinite(thetaLogicalDeg(index)) || ...
            ~isfinite(phiLogicalDeg(index))
        result.status(index) = "NONFINITE_INPUT";
        continue
    end

    q = deg2rad([thetaLogicalDeg(index), phiLogicalDeg(index)]);
    pose = duallink5.kinematics.forwardFiveBar(q, geometry);
    result.status(index) = pose.quality.statusCode;
    if pose.quality.valid
        result.x(index) = pose.points.G(1);
        result.y(index) = pose.points.G(2);
    end
end
result.units.position = "m";
result.units.inputAngle = "deg";
end
