function reference = prepareAssemblyTopologyReference( ...
        referenceAssembly, lengthTolerance)
if ~isnumeric(lengthTolerance) || ~isreal(lengthTolerance) || ...
        ~isscalar(lengthTolerance) || ~isfinite(lengthTolerance) || ...
        lengthTolerance <= 0
    invalidTopology();
end

try
    reference.signature = assemblyTopologySignature( ...
        referenceAssembly, double(lengthTolerance));
catch exception
    if strcmp(exception.identifier, ...
            'duallink5:validation:InvalidAssemblyTopology')
        rethrow(exception)
    end
    invalidTopology();
end
reference.lengthTolerance = double(lengthTolerance);
end

function invalidTopology()
error('duallink5:validation:InvalidAssemblyTopology', ...
    'Assemblies must contain finite complete assembly points.');
end
