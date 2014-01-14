%% Set the appropriate folders

% modify these variables to point to folders on your filesystem

% path to folder where it finds the pcd files
data_folder = '/home/nbore/Data/primitive2/pcd/';

% path to folder to save the dot and index files
graph_folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/dot_graphs/';

% add the gspan command
addpath '../gboost-0.1.1/bin';


%% Call test_graphs to extract primitives and create dot graph files

ld_path = getenv('LD_LIBRARY_PATH');
setenv('LD_LIBRARY_PATH', '');

result = system(['../bin/test_graph "' data_folder '" "' graph_folder '"']);

setenv('LD_LIBRARY_PATH', ld_path);

%% Convert the dot files from test_graph to a matlab graph format
% store in graph_folder
convert_graphs(graph_folder, 'matgraphs.mat');

%% Read the positions and store them in a matlab format
% store in graph_folder
convert_positions(data_folder, graph_folder, 'positions.mat');

%% Load the converted graph

graphfile = [graph_folder 'matgraphs.mat'];
load(graphfile)

%% Load the converted positions and the map

posfile = [graph_folder 'positions.mat'];
load(posfile)
mapfile = [graph_folder 'map.mat'];
load(mapfile)

%% Run the basic gspan analysis

min_nodes = 4;
[subg, count, GY, indices, node_indices] = gspan(G, 30, [min_nodes min_nodes]);
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

ind = 18;

m = length(indices{ind});
for i = 1:m
    fileind = indices{ind}(i) - 1;
    index = [graph_folder sprintf('indices%.6d.txt', fileind)]
    pcdfile = [data_folder sprintf('cloud%.6d.pcd', fileind)]
    display_graph(pcdfile, index, node_indices{ind}(:, i), length(G{fileind+1}.nodelabels));
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

for i = 1:n
    filename = [graph_folder 'subg' sprintf('%.6d', i-1) '.dot'];
    mat2dot(subg{i}, filename);
end

%% Convert the dot files to images, show the graphs

for i = 1:n
    i
    filename = [graph_folder 'subg' sprintf('%.6d', i-1) '.dot'];
    system(['dot -Tpng ' filename ' > ' graph_folder 'test.png']);
    system(['gvfs-open ' graph_folder 'test.png']);
    pause
end