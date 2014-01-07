%%

% convert the dot files from test_graphs to a matlab graph format,
% store in /home/nbore/Workspace/mapping_library/graph_primitives/graphs/matgraphs.mat
convert_graphs();

%%

load '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/matgraphs.mat'

%%

cluster_edges(G);


%%

addpath '/home/nbore/Installs/gboost-0.1.1/bin';
folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/';

[subg, count, GY, indices, node_indices] = gspan(G, 10, [6 0]);
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

viewer = '/home/nbore/Workspace/mapping_library/graph_primitives/bin/display_graph';
data_folder = '/home/nbore/Data/Primitives\ Forward/pcd/';
ld_path = getenv('LD_LIBRARY_PATH');
setenv('LD_LIBRARY_PATH', '');

% show all the partitioned clouds for one extracted graph
ind = 1;
m = length(indices{ind});
for i = 1:m
    fileind = indices{ind}(i) - 1;
    index = [folder sprintf('indices%.6d.txt', fileind)]
    pcdfile = [data_folder sprintf('cloud%.6d.pcd', fileind)]
    ii = node_indices{ind}(:, i);
    gs = length(G{fileind + 1}.nodelabels)
    ordering = zeros(1, gs);
    for j = 1:length(ii)
        ordering(ii(j)) = j;
    end
    os = '"';
    for j = 1:gs
        os = [os ' ' num2str(ordering(j))];
    end
    os = [os '"'];
    result = system([viewer ' ' pcdfile ' ' index ' ' os], '-echo');
end

setenv('LD_LIBRARY_PATH', ld_path);

%%

for i = 1:n
    vim = mean(V{i}, 2);
    vi = V{i} - vim*ones(1, size(V{i}, 2));
    [U, S, VV] = svd(vi');
    vi = VV'*vi;
    plot3(vi(1,:), vi(2,:), vi(3,:), '*')
    pause
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