%% Set the appropriate folders

% modify this variables to point to folders on your filesystem

% path to folder where it finds the pcd files
data_folder = '/home/nbore/Data/Primitives Forward/pcd/';

% path to folder to save the dot and index files
graph_folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/temp';

% add the gspan command
addpath '../gboost-0.1.1/bin';


%% Call test_graphs to extract primitives and create dot graph files

ld_path = getenv('LD_LIBRARY_PATH');
setenv('LD_LIBRARY_PATH', '');

result = system(['../bin/test_graph "' data_folder '" "' graph_folder '"']);

setenv('LD_LIBRARY_PATH', ld_path);

%% Convert the dot files from test_graph to a matlab graph format,
% store in /home/nbore/Workspace/mapping_library/graph_primitives/graphs/matgraphs.mat
folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/';
convert_graphs(folder, 'matgraphs.mat');

%% Load the converted graph

folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/';
graphfile = [folder 'matgraphs.mat'];
load(graphfile)

%% Show the histogram of the plane-plane edges

A = [];

n = length(G);
for i = 1:n
    if G{i}.edgeangles
        A = [A; G{i}.edgeangles];
    end
end

hist(A, 20);


%% Run the basic gspan analysis

addpath '/home/nbore/Installs/gboost-0.1.1/bin';

min_nodes = 4;
[subg, count, GY, indices, node_indices] = gspan(G, 15, [min_nodes 0]);
n = length(subg);

%% Filter base on number of edges

min_edges = 5;
assign_index = 1;
lensubg = length(subg);

i = 1;
while i <= lensubg
    if size(subg{i}.edges, 1) < min_edges
        subg(i) = [];
        count(i) = [];
        GY(:, i) = [];
        indices(i) = [];
        node_indices(i) = [];
        lensubg = lensubg - 1;
    else
        i = i + 1;
    end
end

n = length(subg);

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

%% Construct the vector spaces over continuous node pars

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

%% Show all the partitioned clouds for one extracted graph

ind = 2;

data_folder = '/home/nbore/Data/Primitives\ Forward/pcd/';

m = length(indices{ind});
for i = 1:m
    fileind = indices{ind}(i) - 1;
    index = [folder sprintf('indices%.6d.txt', fileind)]
    pcdfile = [data_folder sprintf('cloud%.6d.pcd', fileind)]
    display_graph(pcdfile, index, node_indices{ind}(:, i), length(G{fileind+1}.nodelabels));
end

%% Alternative way that might crash less often for some reason

ind = 1;

viewer = '/home/nbore/Workspace/mapping_library/graph_primitives/bin/display_graph';
data_folder = '/home/nbore/Data/Primitives\ Forward/pcd/';
ld_path = getenv('LD_LIBRARY_PATH');
setenv('LD_LIBRARY_PATH', '');

% show all the partitioned clouds for one extracted graph
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

%% Show an eigen analysis of the clustering problem

for i = 1:n
    vim = mean(V{i}, 2);
    vi = V{i} - vim*ones(1, size(V{i}, 2));
    [U, S, VV] = svd(vi');
    vi = VV'*vi;
    plot3(vi(1,:), vi(2,:), vi(3,:), '*')
    pause
end

%% Find connected components

i = 1;
while i < n
    connected = connected_components(V{i}, 0.2);
    maxc = 0;
    p = [];
    for j = 1:length(connected)
        if length(connected{j}) > maxc
            p = connected{j};
            maxc = length(p);
        end
    end
    if maxc < 10
        subg(i) = [];
        count(i) = [];
        GY(:, i) = [];
        indices(i) = [];
        node_indices(i) = [];
        n = n - 1;
    else
        indices{i} = indices{i}(p);
        node_indices{i} = node_indices{i}(:, p);
        i = i + 1;
    end
end

%% Convert the graphs into dot files

% [v, ind] = sort(count, 'descend');
% count = v;
% temp = cell(size(subg));
% for i = 1:n
%     temp{i} = subg{ind(i)};
% end
% subg = temp;
% clear temp
% GY = GY(:, ind);

for i = 1:n
    filename = [folder 'subg' sprintf('%.6d', i-1) '.dot'];
    mat2dot(subg{i}, filename);
end

%% Convert the dot files to images, show the graphs

for i = 1:n
    i
    filename = [folder 'subg' sprintf('%.6d', i-1) '.dot'];
    system(['dot -Tpng ' filename ' > ' folder 'test.png']);
    system(['gvfs-open ' folder 'test.png']);
    pause
end