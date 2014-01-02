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

[subg, count, GY] = gspan(G, 10, [5 0]);
[v, ind] = sort(count, 'descend');
count = v;
temp = cell(size(subg));
n = length(subg);
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