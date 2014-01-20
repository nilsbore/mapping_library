function show_positions_in_map(map, i, indices, positions)

horizon = 2;

P = positions(:, 1:2)';
P(2, :) = -P(2, :);
minp = min(P, [], 2);
maxp = max(P, [], 2);

angles = positions(indices{i}, 3)';
offset = [cos(angles); sin(angles)];
P = P(:, indices{i}) + horizon*offset;

[m, n] = size(map);
dist = maxp - minp;

l = length(P);
P = P - minp*ones(1, l);
P = P.*(([n; m]./dist)*ones(1, l));

% this is just a heuristic to find a transformation that looks good
%P = (P + [110; -100]*ones(1, l)).*([0.75; 0.47]*ones(1, l))  + [7; 140]*ones(1, l);
P = (P + [-311; -272]*ones(1, l)).*([0.75;0.45]*ones(1, l))  + [323; 220]*ones(1, l);

imshow(map)
hold on
%plot(P(1, :), P(2, :), '*r', 'MarkerSize', 4)

Q = positions(:, 1:2)';
Q(2, :) = -Q(2, :);
Q = Q(:, indices{i});

Q = Q - minp*ones(1, l);
Q = Q.*(([n; m]./dist)*ones(1, l));

% this is just a heuristic to find a transformation that looks good
%Q = (Q + [110; -100]*ones(1, l)).*([0.75; 0.47]*ones(1, l))  + [7; 140]*ones(1, l);
Q = (Q + [-311; -272]*ones(1, l)).*([0.75;0.45]*ones(1, l))  + [323; 220]*ones(1, l);

%angle = atan(); % offset through focal length
len = 17*horizon*326/566;

x = zeros(1, 3);
y = zeros(1, 3);
for j = 1:l
    o = P(:, j) - Q(:, j);
    o = [o(2); -o(1)];
    o = len/norm(o)*o;
    x(1) = Q(1, j);
    y(1) = Q(2, j);
    x(2) = P(1, j) + o(1);
    y(2) = P(2, j) + o(2);
    x(3) = P(1, j) - o(1);
    y(3) = P(2, j) - o(2);
    fill(x, y, 'r', 'FaceAlpha', 0.15, 'EdgeAlpha', 0)
end

plot(Q(1, :), Q(2, :), 'o', 'MarkerSize', 6, 'MarkerFaceColor', 'b')

end