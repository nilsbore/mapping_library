%% Show the histogram of the plane-plane edges

A = [];

n = length(G);
for i = 1:n
    if G{i}.edgeangles
        A = [A; G{i}.edgeangles];
    end
end

hist(A, 20);

%% Check that the correct indices are found

for i = 1:n
    [nodes, found] = size(node_indices{i});
    for j = 1:found
        for k = 1:nodes
            l1 = subg{i}.nodelabels(k);
            l2 = G{indices{i}(j)}.nodelabels(node_indices{i}(k, j));
            if l1 ~= l2
                l1
                l2
            end
        end
    end
end

%% Show an eigen analysis of the clustering problem

for i = 1:n
    vim = mean(V{i}, 2);
    vi = V{i} - vim*ones(1, size(V{i}, 2));
    [U, S, VV] = svd(vi');
    vi = VV'*vi;
    plot3(vi(1,:), vi(2,:), vi(3,:), '*')
    pause
end