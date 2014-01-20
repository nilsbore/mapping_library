%% Set the appropriate folders

% modify these variables to point to folders on your filesystem

% path to folder where it finds the pcd files
data_folder = '/home/nbore/Data/primitive2/pcd/';

% path to folder to save the dot and index files
graph_folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/dot_graphs2/';

% add the gspan command
addpath '../gboost-0.1.1/bin';

% get the indices of planes and cylinders
p = primitives;
pind = cellfind(p, 'Plane');
cind = cellfind(p, 'Cylinder');
sphind = cellfind(p, 'Sphere');

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
[subg, count, GY, indices, node_indices] = gspan(G, 30, [min_nodes 0]);
n = length(subg);

%% Filter base on number of edges

min_edges = 4;
assign_index = 1;
lensubg = length(subg);

i = 1;
counter = 1;
while counter < n %i <= lensubg
    i
    counter
    adjedges = sum(subg{counter}.edges(:, 3) < 4);
    faredges = sum(subg{counter}.edges(:, 3) > 3);%size(subg{i}.edges, 1) - adjedges;
    m = length(subg{counter}.nodelabels);
    maxedges = m*(m - 1)/2;
    if (adjedges < min_edges) || (faredges +  adjedges < maxedges)
        %subg(i) = [];
        %count(i) = [];
        %GY(:, i) = [];
        %indices(i) = [];
        %node_indices(i) = [];
        lensubg = lensubg - 1;
    else
        subg{i} = subg{counter};
        count(i) = count(counter);
        GY(:, i) = GY(:, counter);
        indices{i} = indices{counter};
        node_indices{i} = node_indices{counter};
        i = i + 1;
    end
    counter = counter + 1;
end

subg(i:end) = [];
count(i:end) = [];
GY(:, i:end) = [];
indices(i:end) = [];
node_indices(i:end) = [];
n = length(subg);

%% Save the frequent subgraphs

freqfile = [graph_folder 'matsubgraphs.mat'];
save(freqfile, 'subg', 'count', 'GY', 'indices', 'node_indices');

%% Load the frequent subgraphs

freqfile = [graph_folder 'matsubgraphs.mat'];
load(freqfile)
n = length(subg);

%% Show distribution of plane and cylinder sizes

m = length(G);

planesizes = [];
cylindersizes = [];

p = primitives;
pind = cellfind(p, 'Plane');
cind = cellfind(p, 'Cylinder');

for i = 1:m
    ll = G{i}.nodelabels;
    for j = 1:length(ll)
       if ll(j) == pind
           planesizes = [planesizes G{i}.nodesizes(j)];
       elseif ll(j) == cind
           cylindersizes = [cylindersizes G{i}.nodesizes(j)];
       end
    end
end

hist(planesizes, 30)
figure
hist(cylindersizes, 30)

planestd = std(planesizes)
cylinderstd = std(cylindersizes)
planemean = mean(planesizes);
cylindermean = mean(cylindersizes);

%% Construct the vector spaces over continuous node pars

V = cell(1, n);

for i = 1:n
    [nodes, found] = size(node_indices{i});
    V{i} = zeros(nodes, found);
    for j = 1:found
        for k = 1:nodes
            pdivide = 1;
            pmin = 0;
            i
            size(subg)
            k
            size(subg{i}.nodelabels)
            if subg{i}.nodelabels(k) == pind
                pdivide = planestd;
                pmin = planemean;
            elseif subg{i}.nodelabels(k) == cind
                pdivide = cylinderstd;
                pmin = cylindermean;
            end
            V{i}(k, j) = (G{indices{i}(j)}.nodesizes(node_indices{i}(k, j))-pmin)/pdivide;
        end
    end
end

%% Show all the partitioned clouds for one extracted graph

ind = 7;
screenshot_folder = [graph_folder sprintf('%.6d/', ind)];
mkdir(screenshot_folder)

m = length(indices{ind});
for i = 1:m
    fileind = indices{ind}(i) - 1;
    index = [graph_folder sprintf('indices%.6d.txt', fileind)]
    pcdfile = [data_folder sprintf('cloud%.6d.pcd', fileind)]
    screenshot = [screenshot_folder sprintf('graph%.6d.png', fileind)]
    display_graph(pcdfile, index, screenshot, node_indices{ind}(:, i), length(G{fileind+1}.nodelabels));
end

%% Find connected components

i = 1;
while i < n
    i
    connected = connected_components(V{i}, 0.2);
    maxc = 0;
    p = [];
    for j = 1:length(connected)
        if length(connected{j}) > maxc
            p = connected{j};
            maxc = length(p);
        end
    end
    if maxc < 5
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