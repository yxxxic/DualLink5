function comparison = compareAssemblyTopology( ...
        candidateAssembly, referenceAssembly, geometry)
geometry = duallink5.model.validateGeometry(geometry);

try
    candidateSignature = assemblyTopologySignature( ...
        candidateAssembly, geometry.tolerance.length);
    referenceSignature = assemblyTopologySignature( ...
        referenceAssembly, geometry.tolerance.length);
catch exception
    if strcmp(exception.identifier, ...
            'duallink5:validation:InvalidAssemblyTopology')
        rethrow(exception)
    end
    error('duallink5:validation:InvalidAssemblyTopology', ...
        'Assemblies must contain finite complete assembly points.');
end

comparison.candidateSignature = candidateSignature;
comparison.referenceSignature = referenceSignature;
comparison.addedCrossings = setdiff( ...
    candidateSignature, referenceSignature, 'rows');
comparison.removedCrossings = setdiff( ...
    referenceSignature, candidateSignature, 'rows');
comparison.compatible = isempty(comparison.addedCrossings) && ...
    isempty(comparison.removedCrossings);
if comparison.compatible
    comparison.statusCode = "OK";
else
    comparison.statusCode = "ASSEMBLY_TOPOLOGY_MISMATCH";
end
end
