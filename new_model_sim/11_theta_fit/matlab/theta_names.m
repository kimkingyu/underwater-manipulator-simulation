function [names, units] = theta_names(n, hull_K)
%THETA_NAMES  Names and units of the 87 entries of theta (see Y_uvms); 105 with hull_K (K_rk, 88:105).
if nargin < 1, n = 3; end
if nargin < 2, hull_K = false; end
dof = {'u', 'v', 'w', 'p', 'q', 'r'};
names = {}; units = {};
for k = 1:6
    names{end + 1} = ['MA_' dof{k}]; units{end + 1} = ternary(k <= 3, 'kg', 'kg*m^2');
end
for k = 1:6
    names{end + 1} = ['Dl_' dof{k}]; units{end + 1} = ternary(k <= 3, 'N*s/m', 'N*m*s/rad');
end
for k = 1:6
    names{end + 1} = ['Dq_' dof{k}]; units{end + 1} = ternary(k <= 3, 'N*s^2/m^2', 'N*m*s^2/rad^2');
end
ax = 'xyz';
for i = 1:n
    for k = 1:3
        names{end + 1} = sprintf('L%d_A%c', i, ax(k)); units{end + 1} = 'kg';
    end
    for k = 1:3
        names{end + 1} = sprintf('L%d_Dq%c', i, ax(k)); units{end + 1} = 'N*s^2/m^2';
    end
end
for j = 1:n
    names{end + 1} = sprintf('Fv%d', j); units{end + 1} = 'N*m*s/rad';
end
for j = 1:n
    names{end + 1} = sprintf('Fc%d', j); units{end + 1} = 'N*m';
end
mom = 'KMN'; tr = 'uvw';
for k = 1:3
    for j = 1:3
        names{end + 1} = sprintf('C_%c%c', mom(j), tr(k)); units{end + 1} = 'N*s';
    end
end
for k = 1:3
    for j = 1:3
        names{end + 1} = sprintf('Cq_%c%c', mom(j), tr(k)); units{end + 1} = 'N*s^2/m';
    end
end
for i = 1:n
    for k = 1:3
        for j = 1:3
            names{end + 1} = sprintf('L%d_E%c%c', i, ax(j), ax(k)); units{end + 1} = 'N*s^2/m';
        end
    end
end
if hull_K
    row = {'X', 'Y', 'Z', 'K', 'M', 'N'};
    for r = 1:6
        for k = 1:3
            names{end + 1} = sprintf('K_%s%c', row{r}, tr(k));
            units{end + 1} = ternary(r <= 3, 'N*s^2/m^2', 'N*s^2/m');
        end
    end
end
names = names(:); units = units(:);
end

function o = ternary(c, a, b)
if c, o = a; else, o = b; end
end
