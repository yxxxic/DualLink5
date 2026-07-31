function points = computeParallelPoints(A, C, D, E, geometry)
A = A(:); C = C(:); D = D(:); E = E(:);
tolerance = geometry.tolerance.length;

uAE = unitVector(E - A, tolerance, 'AE');
uCD = unitVector(D - C, tolerance, 'CD');
L = geometry.parallel.lengths;
direction = geometry.parallel.direction;

alphaShift = direction.alphaAlongLink * L.E_Palpha1 * uAE;
points.Palpha1 = E + alphaShift;
points.Palpha2 = D + alphaShift;
alphaSide = direction.alphaSide * L.Palpha1_Palpha4 * uAE;
points.Palpha3 = points.Palpha2 + alphaSide;
points.Palpha4 = points.Palpha1 + alphaSide;

betaShift = direction.betaAlongLink * L.D_Pbeta2 * uCD;
points.Pbeta1 = E + betaShift;
points.Pbeta2 = D + betaShift;
betaSide = direction.betaSide * L.Pbeta2_Pbeta3 * uCD;
points.Pbeta3 = points.Pbeta2 + betaSide;
points.Pbeta4 = points.Pbeta1 + betaSide;
end

function unit = unitVector(vector, tolerance, label)
lengthValue = norm(vector);
if lengthValue <= tolerance
    error('duallink5:kinematics:DegenerateDirection', ...
        '%s direction is degenerate.', label);
end
unit = vector / lengthValue;
end
