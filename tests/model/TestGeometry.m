classdef TestGeometry < matlab.unittest.TestCase
    methods (TestMethodSetup)
        function addProjectSource(~)
            testDir = fileparts(mfilename('fullpath'));
            projectRoot = fileparts(fileparts(testDir));
            run(fullfile(projectRoot, 'startup.m'));
        end
    end

    methods (Test)
        function defaultLengthsUseSIAndApprovedMapping(testCase)
            g = duallink5.model.defaultGeometry();

            testCase.verifyEqual(g.links.link1, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link2, 62e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link3, 69e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link4, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.links.link5, 80e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(g.units.length, "m");
            testCase.verifyEqual(g.units.angle, "rad");
            testCase.verifyEqual( ...
                g.collision.parallelSharedClearance, 7.5e-3, ...
                'AbsTol', 1e-15);
        end

        function topologyReferenceUsesApprovedPose(testCase)
            g = duallink5.model.defaultGeometry();

            testCase.verifyEqual(g.analysis.referenceQ, deg2rad([85, 30]), ...
                'AbsTol', 1e-15);
        end

        function couplingWarningAngleUsesApprovedValue(testCase)
            geometry = duallink5.model.defaultGeometry();

            testCase.verifyEqual( ...
                geometry.analysis.couplingWarningAngle, ...
                deg2rad(10), 'AbsTol', 1e-15);
        end

        function invalidCouplingWarningAnglesAreRejected(testCase)
            geometry = duallink5.model.defaultGeometry();
            invalid = {0, -1, pi / 2, Inf, NaN, [0.1, 0.2]};

            for index = 1:numel(invalid)
                candidate = geometry;
                candidate.analysis.couplingWarningAngle = invalid{index};
                testCase.verifyError( ...
                    @()duallink5.model.validateGeometry(candidate), ...
                    'duallink5:model:InvalidGeometry');
            end
        end

        function parallelDimensionsUseEndpointNames(testCase)
            g = duallink5.model.defaultGeometry();
            L = g.parallel.lengths;

            testCase.verifyEqual(L.E_Palpha1, 30e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.Palpha1_Palpha4, 60e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.D_Pbeta2, 30e-3, 'AbsTol', 1e-15);
            testCase.verifyEqual(L.Pbeta2_Pbeta3, 60e-3, 'AbsTol', 1e-15);
        end

        function invalidUnitIsRejected(testCase)
            g = duallink5.model.defaultGeometry();
            g.units.length = "mm";
            testCase.verifyError(@() duallink5.model.validateGeometry(g), ...
                'duallink5:model:InvalidUnits');
        end

        function missingDirectionFieldIsRejected(testCase)
            g=duallink5.model.defaultGeometry();
            g.parallel.direction=rmfield(g.parallel.direction,'betaSide');
            testCase.verifyError(@()duallink5.model.validateGeometry(g), ...
                'duallink5:model:InvalidGeometry');
        end

        function malformedStructCategoriesUseGeometryError(testCase)
            g = duallink5.model.defaultGeometry();
            cases = {
                42
                repmat(g, 1, 2)
                TestGeometry.withoutField(g, {'links'})
                TestGeometry.withValue(g, {'links'}, 42)
                TestGeometry.withValue(g, {'links'}, repmat(g.links, 1, 2))
                TestGeometry.withValue(g, {'parallel', 'lengths'}, 42)
                TestGeometry.withValue(g, {'parallel', 'direction'}, ...
                    repmat(g.parallel.direction, 1, 2))
                TestGeometry.withValue(g, {'assembly'}, 42)
                TestGeometry.withValue(g, {'tolerance'}, 42)
                TestGeometry.withValue(g, {'analysis'}, 42)
                TestGeometry.withValue(g, {'collision'}, 42)
                };

            TestGeometry.verifyErrors(testCase, cases, ...
                'duallink5:model:InvalidGeometry');
        end

        function invalidScalarValuesUseGeometryError(testCase)
            g = duallink5.model.defaultGeometry();
            cases = {
                TestGeometry.withValue(g, {'links', 'link1'}, 0)
                TestGeometry.withValue(g, {'links', 'link2'}, NaN)
                TestGeometry.withValue(g, ...
                    {'parallel', 'lengths', 'E_Palpha1'}, -1)
                TestGeometry.withValue(g, ...
                    {'parallel', 'lengths', 'Pbeta2_Pbeta3'}, Inf)
                TestGeometry.withValue(g, ...
                    {'parallel', 'direction', 'alphaSide'}, 0)
                TestGeometry.withValue(g, ...
                    {'parallel', 'direction', 'betaAlongLink'}, 2)
                TestGeometry.withValue(g, ...
                    {'assembly', 'defaultBranch'}, 0)
                TestGeometry.withValue(g, ...
                    {'tolerance', 'length'}, 0)
                TestGeometry.withValue(g, ...
                    {'tolerance', 'residual'}, Inf)
                TestGeometry.withValue(g, ...
                    {'analysis', 'thetaRange'}, [1, 1])
                TestGeometry.withValue(g, ...
                    {'analysis', 'phiRange'}, [0; 1])
                TestGeometry.withValue(g, ...
                    {'analysis', 'phiRange'}, [0, Inf])
                };

            TestGeometry.verifyErrors(testCase, cases, ...
                'duallink5:model:InvalidGeometry');
        end

        function malformedTopologyReferencesAreRejected(testCase)
            g = duallink5.model.defaultGeometry();
            cases = {
                TestGeometry.withValue(g, {'analysis', 'referenceQ'}, [85, 30, 0])
                TestGeometry.withValue(g, {'analysis', 'referenceQ'}, [85; 30])
                TestGeometry.withValue(g, {'analysis', 'referenceQ'}, [85, NaN])
                TestGeometry.withValue(g, {'analysis', 'referenceQ'}, "85,30")
                };

            TestGeometry.verifyErrors(testCase, cases, ...
                'duallink5:model:InvalidGeometry');
        end

        function invalidCollisionFieldsUseGeometryError(testCase)
            g = duallink5.model.defaultGeometry();
            cases = {
                TestGeometry.withValue(g, {'collision', 'radius'}, 42)
                TestGeometry.withValue(g, {'collision', 'layerOffset'}, 42)
                TestGeometry.withValue(g, {'collision', 'clearance'}, -1)
                TestGeometry.withValue(g, {'collision', 'clearance'}, NaN)
                TestGeometry.withValue(g, ...
                    {'collision', 'parallelSharedClearance'}, -1)
                TestGeometry.withValue(g, ...
                    {'collision', 'parallelSharedClearance'}, NaN)
                TestGeometry.withValue(g, ...
                    {'collision', 'exemptPairs'}, {'A', 'B'})
                TestGeometry.withValue(g, ...
                    {'collision', 'exemptPairs'}, strings(2, 1))
                };

            TestGeometry.verifyErrors(testCase, cases, ...
                'duallink5:model:InvalidGeometry');
        end

        function collisionStructArraysAreRejected(testCase)
            g = duallink5.model.defaultGeometry();
            cases = {
                TestGeometry.withValue(g, {'collision', 'radius'}, ...
                    repmat(struct(), 1, 2))
                TestGeometry.withValue(g, {'collision', 'layerOffset'}, ...
                    repmat(struct(), 1, 2))
                };

            TestGeometry.verifyErrors(testCase, cases, ...
                'duallink5:model:InvalidGeometry');
        end

        function collisionExemptPairsMustBeMatrix(testCase)
            g = duallink5.model.defaultGeometry();
            g.collision.exemptPairs = strings(1, 2, 2);

            testCase.verifyError( ...
                @() duallink5.model.validateGeometry(g), ...
                'duallink5:model:InvalidGeometry');
        end

        function invalidUnitsUseUnitError(testCase)
            g = duallink5.model.defaultGeometry();
            cases = {
                TestGeometry.withoutField(g, {'units'})
                TestGeometry.withValue(g, {'units'}, 42)
                TestGeometry.withValue(g, {'units'}, repmat(g.units, 1, 2))
                TestGeometry.withoutField(g, {'units', 'length'})
                TestGeometry.withValue(g, {'units', 'length'}, struct())
                TestGeometry.withValue(g, ...
                    {'units', 'length'}, ["m", "m"])
                TestGeometry.withValue(g, {'units', 'angle'}, "deg")
                };

            TestGeometry.verifyErrors(testCase, cases, ...
                'duallink5:model:InvalidUnits');
        end

        function invalidMetadataUsesGeometryError(testCase)
            g = duallink5.model.defaultGeometry();
            cases = {
                TestGeometry.withoutField(g, {'version'})
                TestGeometry.withValue(g, {'version'}, "")
                TestGeometry.withValue(g, {'version'}, ["1", "2"])
                TestGeometry.withValue(g, {'version'}, struct())
                TestGeometry.withoutField(g, {'convention'})
                TestGeometry.withValue(g, {'convention'}, "")
                TestGeometry.withValue(g, {'convention'}, struct())
                };

            TestGeometry.verifyErrors(testCase, cases, ...
                'duallink5:model:InvalidGeometry');
        end

        function scalarConvertibleMetadataIsAccepted(testCase)
            g = duallink5.model.defaultGeometry();
            g.version = 42;
            g.convention = true;

            actual = duallink5.model.validateGeometry(g);

            testCase.verifyEqual(actual.version, 42);
            testCase.verifyEqual(actual.convention, true);
        end

        function startupPreservesCallerVariables(testCase)
            testDir = fileparts(mfilename('fullpath'));
            startupFile = fullfile( ...
                fileparts(fileparts(testDir)), 'startup.m');
            projectRoot = "caller-project-root";
            sourceDir = "caller-source-dir";
            pathEntries = "caller-path-entries";

            run(startupFile);

            testCase.verifyEqual(projectRoot, "caller-project-root");
            testCase.verifyEqual(sourceDir, "caller-source-dir");
            testCase.verifyEqual(pathEntries, "caller-path-entries");
        end

        function startupIsIdempotent(testCase)
            testDir = fileparts(mfilename('fullpath'));
            startupFile = fullfile( ...
                fileparts(fileparts(testDir)), 'startup.m');
            originalPath = path;

            run(startupFile);
            run(startupFile);

            testCase.verifyEqual(path, originalPath);
        end
    end

    methods (Static, Access=private)
        function verifyErrors(testCase, cases, errorId)
            for index = 1:numel(cases)
                value = cases{index};
                testCase.verifyError( ...
                    @() duallink5.model.validateGeometry(value), errorId);
            end
        end

        function result = withValue(result, fieldPath, replacement)
            fieldName = fieldPath{1};
            if isscalar(fieldPath)
                result.(fieldName) = replacement;
                return
            end

            child = result.(fieldName);
            child = TestGeometry.withValue( ...
                child, fieldPath(2:end), replacement);
            result.(fieldName) = child;
        end

        function result = withoutField(result, fieldPath)
            fieldName = fieldPath{1};
            if isscalar(fieldPath)
                result = rmfield(result, fieldName);
                return
            end

            child = result.(fieldName);
            child = TestGeometry.withoutField(child, fieldPath(2:end));
            result.(fieldName) = child;
        end
    end
end
