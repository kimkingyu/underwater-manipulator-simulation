function X = hydro_surrogate_inputs(q, qd, qdd, vc)
%HYDRO_SURROGATE_INPUTS Validate and assemble N-by-12 inputs in SI units.
% q/qd/qdd must have identical N-by-3 shape; vc is N-by-3 or 1-by-3.
args = {q, qd, qdd, vc};
names = {'q', 'qd', 'qdd', 'vc'};
for k = 1:4
    validateattributes(args{k}, {'numeric'}, ...
        {'real','finite','2d','ncols',3,'nonempty'}, mfilename, names{k});
end
n = size(q,1);
if size(qd,1) ~= n || size(qdd,1) ~= n || ~ismember(size(vc,1), [1 n])
    error('hydro:InputSize', 'q/qd/qdd must be N-by-3; vc must be 1-by-3 or N-by-3.');
end
if size(vc,1) == 1
    vc = repmat(vc,n,1);
end
X = double([q, qd, qdd, vc]);
end
