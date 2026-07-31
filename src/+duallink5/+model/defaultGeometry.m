function geometry = defaultGeometry()
geometry.version = "1.0.0";

geometry.links.link1 = 80e-3;  % AE
geometry.links.link2 = 62e-3;  % BC
geometry.links.link3 = 69e-3;  % CD
geometry.links.link4 = 80e-3;  % DE
geometry.links.link5 = 80e-3;  % AB

geometry.parallel.lengths.E_Palpha1 = 30e-3;
geometry.parallel.lengths.Palpha1_Palpha4 = 60e-3;
geometry.parallel.lengths.D_Pbeta2 = 30e-3;
geometry.parallel.lengths.Pbeta2_Pbeta3 = 60e-3;

geometry.parallel.direction.alphaAlongLink = -1;
geometry.parallel.direction.alphaSide = 1;
geometry.parallel.direction.betaAlongLink = -1;
geometry.parallel.direction.betaSide = 1;

geometry.assembly.defaultBranch = int8(-1);

geometry.tolerance.length = 1e-9;
geometry.tolerance.residual = 1e-9;
geometry.tolerance.symmetryPosition = 1e-8;
geometry.tolerance.symmetryAngle = 1e-8;
geometry.tolerance.singularityCondition = 1e10;

geometry.analysis.thetaRange = [0, pi];
geometry.analysis.phiRange = [0, pi/2];

geometry.collision.radius = struct();
geometry.collision.layerOffset = struct();
geometry.collision.clearance = 0;
geometry.collision.exemptPairs = strings(0, 2);

geometry.units.length = "m";
geometry.units.angle = "rad";
geometry.convention = ...
    "A-origin; +x=A-to-B; +y=mechanism-interior; q=[theta,phi]";

geometry = duallink5.model.validateGeometry(geometry);
end
