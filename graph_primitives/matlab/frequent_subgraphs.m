%%

% convert the dot files from test_graphs to a matlab graph format,
% store in /home/nbore/Workspace/mapping_library/graph_primitives/graphs/matgraphs.mat
convert_graphs();

%%

load '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/matgraphs.mat'
cluster_edges(G);


%%

addpath '/home/nbore/Installs/gboost-0.1.1/bin';
folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/';

[subg, count, GY, indices, node_indices] = gspan(G, 10, [5 0]);
n = length(subg);

%%

% check that the correct indices are found
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

%%

% construct the vector spaces
V = cell(1, n);

for i = 1:n
    [nodes, found] = size(node_indices{i});
    V{i} = zeros(nodes, found);
    for j = 1:found
        for k = 1:nodes
            V{i}(k, j) = G{indices{i}(j)}.nodesizes(node_indices{i}(k, j));
        end
    end
end

%%

for i = 1:n
    
end

%%

[v, ind] = sort(count, 'descend');
count = v;
temp = cell(size(subg));
for i = 1:n
    temp{i} = subg{ind(i)};
end
subg = temp;
clear temp
GY = GY(:, ind);

% convert the graphs into dot files
for i = 1:n
    filename = [folder 'subg' sprintf('%.6d', i-1) '.dot'];
    mat2dot(subg{i}, filename);
end

%%

% show the graphs
for i = 1:n
    filename = [folder 'subg' sprintf('%.6d', i-1) '.dot'];
    system(['dot -Tpng ' filename ' > ' folder 'test.png']);
    system(['gvfs-open ' folder 'test.png']);
    pause
end