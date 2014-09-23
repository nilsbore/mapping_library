%% Set the appropriate folders

% modify these variables to point to folders on your filesystem

% path to folder where it finds the pcd files
data_folder = '/home/nbore/Data/primitiveall/';

% path to folder to save the dot and index files
%graph_folder = '/home/nbore/.ros/final_map3/graphs65/';
graph_folder = '/home/nbore/.ros/final_map3/graphs34/';

% add the gspan command
addpath '../gboost-0.1.1/bin'; 

% get the indices of planes and cylinderscd 
p = primitives;
pind = cellfind(p, 'Plane');
cind = cellfind(p, 'Cylinder');
sphind = cellfind(p, 'Sphere');

%% Show all the partitioned clouds for one extracted graph

screenshot_folder = [graph_folder 'screenshots/'];
mkdir(screenshot_folder)

n = 1;
for i = 0:n
    index = [graph_folder sprintf('indices%.6d.txt', i)]
    pcdfile = [graph_folder sprintf('cloud%.6d.pcd', i)]
    screenshot = [screenshot_folder sprintf('screen%.6d.png', i)]
    display_graph(pcdfile, index, screenshot, 1:length(G{n}.nodelabels)-1, length(G{n}.nodelabels));
end

%% Call test_graphs to extract primitives and create dot graph files

ld_path = getenv('LD_LIBRARY_PATH');
setenv('LD_LIBRARY_PATH', '');

result = system(['../bin/test_graph "' data_folder '" "' graph_folder '"']);

setenv('LD_LIBRARY_PATH', ld_path);

%% Convert the dot files from test_graph to a matlab graph format
% store in graph_folder
convert_graphs2(graph_folder, 'matgraphs2.mat');

%% Read the positions and store them in a matlab format
% store in graph_folder
convert_positions(data_folder, graph_folder, 'positions.mat');

%% Load the converted graph

graphfile = [graph_folder 'matgraphs2.mat'];
load(graphfile)

%% Change the floor nodes to have its own label instead of plane

for i = 1:length(G)
    nodedata = G{i}.nodedata(:, 1:4);
    floorinds = find(all(nodedata == ones(size(nodedata, 1), 1)*[0 0 1 0], 2));
    G{i}.nodelabels(floorinds) = 2; % assuming we only have planes
end

%% Load the converted positions and the map

posfile = [graph_folder 'positions.mat'];
load(posfile)
%mapfile = [graph_folder 'map.mat'];
%load(mapfile)
map = imread('/home/nbore/Data/primitive2/pcd/floorsix.pgm');

%% Segment the graph into groups of maximal cliques

min_nodes = 4;
N = test_cliques(G, min_nodes);

%% Run the basic gspan analysis

min_nodes = 5;%6;
%minsup = 17; % for 6x6
minsup = 30; % for 3x3
% 26 for five?
[subg, count, GY, indices, node_indices] = gspan(G, minsup, [min_nodes min_nodes+1]); %15
n = length(subg);

%% If done maximal clique segmentation before, revert to old indices

[indices, node_indices] = revert_indices(indices, node_indices, N);

%% Save the temp subgraphs

freqfile = [graph_folder 'temp2.mat'];
save(freqfile, 'subg', 'count', 'GY', 'indices', 'node_indices');

%% Load the temp subgraphs

freqfile = [graph_folder 'temp2.mat'];
load(freqfile)
n = length(subg);

%% Remove double counts with nodes representing the same structure
% also filter based on number of adjacent edges

min_edges = 8;

for i = 1:n
    keep_inds = [];
    unique_inds = unique(indices{i});
    for j = 1:length(unique_inds)
        ind = find(indices{i} == unique_inds(j));
        if length(ind) == 1
            keep_inds = [keep_inds ind];
            continue
        end
        A = sort(node_indices{i}(:, ind), 1, 'ascend');
        [~, I] = unique(A', 'rows', 'first');
        keep_inds = [keep_inds ind(I)];
    end
    indices{i} = indices{i}(keep_inds);
    node_indices{i} = node_indices{i}(:, keep_inds);
end

lensubg = length(subg);
i = 1;
counter = 1;
while counter < n %i <= lensubg
    i
    counter
    adjedges = sum(subg{counter}.edges(:, 3) < 4);
    faredges = sum(subg{counter}.edges(:, 3) > 3);
    m = length(subg{counter}.nodelabels);
    maxedges = m*(m - 1)/2;
    if (length(indices{counter}) < minsup) || (adjedges < min_edges)
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

%% Filter base on number of edges

%min_edges = 15;
min_edges = 5;
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
    if (adjedges < min_edges)  || (faredges + adjedges < maxedges)
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

freqfile = [graph_folder 'matsubgraphs2.mat'];
save(freqfile, 'subg', 'count', 'GY', 'indices', 'node_indices');

%% Load the frequent subgraphs

freqfile = [graph_folder 'matsubgraphs2.mat'];
load(freqfile)
n = length(subg);

%% Remove double counts with nodes representing the same structure

for i = 1:n
    keep_inds = [];
    unique_inds = unique(indices{i});
    for j = 1:length(unique_inds)
        ind = find(indices{i} == unique_inds(j));
        if length(ind) == 1
            keep_inds = [keep_inds ind];
            continue
        end
        A = sort(node_indices{i}(:, ind), 1, 'ascend');
        [~, I] = unique(A', 'rows', 'first');
        keep_inds = [keep_inds ind(I)];
    end
    indices{i} = indices{i}(keep_inds);
    node_indices{i} = node_indices{i}(:, keep_inds);
end

%% Show distribution of plane and cylinder sizes

m = length(G);

% for i = 1:m
%     ll = G{i}.nodelabels;
%     for j = 1:length(ll)
%        if ll(j) == pind
%            G{i}.nodesizes(j) = log(0.2 + G{i}.nodesizes(j));
%        end
%     end
% end

planesizes = [];
cylindersizes = [];

p = primitives;
pind = cellfind(p, 'Plane');
cind = cellfind(p, 'Cylinder');

for i = 1:m
    ll = G{i}.nodelabels;
    for j = 1:length(ll)
       if ll(j) == pind
           planesizes = [planesizes log(0.2 + G{i}.nodesizes(j))];
           %planesizes = [planesizes G{i}.nodesizes(j)];
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
            psize = G{indices{i}(j)}.nodesizes(node_indices{i}(k, j));
            if subg{i}.nodelabels(k) == pind
                pdivide = planestd;
                pmin = planemean;
                psize = log(0.2 + psize);
            elseif subg{i}.nodelabels(k) == cind
                pdivide = cylinderstd;
                pmin = cylindermean;
            end
            V{i}(k, j) = (psize - pmin)/pdivide;
        end
    end
end

%% Cluster the graphs

[subg, count, GY, indices, node_indices] = ...
    cluster_graphs(subg, count, GY, indices, node_indices, V, true);
n = length(subg);

%% Show all the partitioned clouds for one extracted graph

ind = 1;
screenshot_folder = [graph_folder sprintf('clustered%.6d/', ind)];
mkdir(screenshot_folder)

m = length(indices{ind});
for i = 1:m
    fileind = indices{ind}(i) - 1;
    index = [graph_folder sprintf('indices%.6d.txt', fileind)]
    pcdfile = [graph_folder sprintf('cloud%.6d.pcd', fileind)]
    screenshot = [screenshot_folder sprintf('graph%.6d_%.3d.png', fileind, i)]
    node_indices_ind = node_indices{ind}(:, i);
    nodedata = G{fileind+1}.nodedata(node_indices_ind, 1:4)
    floorind = find(all(nodedata == ones(size(nodedata, 1), 1)*[0 0 1 0], 2));
    number_nodes = length(G{fileind+1}.nodelabels);
    if isempty(floorind)
        disp 'Floor is not part of graph'
    else
        disp 'Floor found'
        node_indices_ind(floorind) = [];
        number_nodes = number_nodes - 1;
    end
    display_graph(pcdfile, index, screenshot, node_indices_ind, number_nodes);
end

%% Show all the partitioned clouds of one group in the big map

for ind = 1%1:length(subg)
%ind = 14;
screenshot_folder = [graph_folder sprintf('clustered%.6d/', ind)];
mkdir(screenshot_folder)
pcd_file = [graph_folder 'full_cloud.pcd'];
screenshot_file = [screenshot_folder sprintf('full_cloud%.6d.png', ind)];

m = length(indices{ind});
index_files = '"';
map_file = '';
primitive_indices = cell(1, m);
number_primitives = cell(1, m);

for i = 1:m
    fileind = indices{ind}(i) - 1;
    index = [graph_folder sprintf('super_indices%.6d.txt', fileind)];
    if i == 1
        index_files = [index_files index];
    else
        index_files = [index_files ' ' index];
    end
    %index_files
    %pcdfile = [graph_folder sprintf('cloud%.6d.pcd', fileind)]
    %screenshot = [screenshot_folder sprintf('graph%.6d_%.3d.png', fileind, i)]
    node_indices_ind = node_indices{ind}(:, i);
    nodedata = G{fileind+1}.nodedata(node_indices_ind, 1:4);
    floorind = find(all(nodedata == ones(size(nodedata, 1), 1)*[0 0 1 0], 2));
    number_primitives{i} = length(G{fileind+1}.nodelabels);
    if isempty(floorind)
        disp 'Floor is not part of graph'
    else
        disp 'Floor found'
        node_indices_ind(floorind) = [];
        number_primitives{i} = number_primitives{i} - 1;
    end
    primitive_indices{i} = node_indices_ind;
end
index_files = [index_files '"'];

display_graph_map(pcd_file, index_files, screenshot_file, primitive_indices, number_primitives);
end

%% Save the places in the map

savemap = true; % change to false if you just want to view maps

%highlight = [25 50 100];
%highlight = [30 50 70];
%highlight = [25 53 82]; % for rest
highlight = [25 50 82]; % for 2

for i = 1:n
    screenshot_folder = [graph_folder sprintf('clustered%.6d/', i)];
    posmapfile = [screenshot_folder 'posmap.eps'];
    if savemap
        show_positions_in_map(map, i, indices, positions, [], posmapfile);
    else
        show_positions_in_map(map, i, indices, positions, []);
        pause
    end
end

%% Find largest cluster within the graphs

%[subg, count, GY, indices, node_indices] = ...
cluster_graphs(subg, count, GY, indices, node_indices, V, true);

%% Find connected components

i = 10;
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
    system(['dot -Teps ' filename ' > ' graph_folder 'test.eps']);
    system(['gvfs-open ' graph_folder 'test.eps']);
    pause
end