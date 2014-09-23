%% Gspan for finding stuff similar to what the user clicked

picked_folder = [graph_folder 'picked/'];

%% Convert the dot files from test_graph to a matlab graph format
% store in graph_folder
convert_graphs2(picked_folder, 'matgraphs1.mat');

%% Load the converted graph

graphfile = [picked_folder 'matgraphs1.mat'];
load(graphfile)

%% Change the floor nodes to have its own label instead of plane

for i = 1:length(G)
    nodedata = G{i}.nodedata(:, 1:4);
    floorinds = find(all(nodedata == ones(size(nodedata, 1), 1)*[0 0 1 0], 2));
    G{i}.nodelabels(floorinds) = 2; % assuming we only have planes
end

%% Run the basic gspan analysis

min_nodes = 5;
minsup = 2; % for 3x3

% 26 for five?
[areas, count, GY, indices, node_indices] = gspan(G, minsup, [min_nodes min_nodes+1]); %15
n = length(subg);

%% Save the frequent subgraphs

freqfile = [picked_folder 'matsubgraphs1.mat'];
save(freqfile, 'areas', 'count', 'GY', 'indices', 'node_indices');

%% Load the frequent subgraphs

freqfile = [picked_folder 'matsubgraphs1.mat'];
load(freqfile)
n = length(areas);

%% Filter base on number of edges

min_edges = 5;
assign_index = 1;
lensubg = length(areas);

i = 1;
counter = 1;
while counter < n %i <= lensubg
    i
    counter
    adjedges = sum(areas{counter}.edges(:, 3) < 4);
    faredges = sum(areas{counter}.edges(:, 3) > 3);
    m = length(subg{counter}.nodelabels);
    maxedges = m*(m - 1)/2;
    if (adjedges < min_edges)  || (faredges + adjedges < maxedges)
        lensubg = lensubg - 1;
    else
        areas{i} = areas{counter};
        count(i) = count(counter);
        GY(:, i) = GY(:, counter);
        indices{i} = indices{counter};
        node_indices{i} = node_indices{counter};
        i = i + 1;
    end
    counter = counter + 1;
end

areas(i:end) = [];
count(i:end) = [];
GY(:, i:end) = [];
indices(i:end) = [];
node_indices(i:end) = [];
n = length(areas);

%% See how many of the respective doors are in the data
% load the big graph G before doing this, frequent_subgraphs2

m = length(G);
areapresent = false(1, m);
node_indices = {[]};

[~, j] = max(count);
maxedges = min_nodes*(min_nodes-1)/2;
for i = 1:m
    [subgt, countt, GYt, indicest, node_indicest] = gspan({G{i} areas{j}}, 2, [min_nodes min_nodes+1]);
    for k = 1:length(subgt)
        if length(subgt{k}.edges) == maxedges
            areapresent(i) = true;
            I = find(indicest{k} == 1);
            node_indices{1} = [node_indices{1} node_indicest{k}(:, I(1))];
        end
    end
end

indices = {[]};
indices{1} = find(areapresent(1, :));

%% Save the frequent subgraphs

freqfile = [picked_folder 'temp1.mat'];
save(freqfile, 'areas', 'count', 'indices', 'node_indices');

%% Show all the partitioned clouds of one group in the big map

ind = 1;
screenshot_folder = [picked_folder sprintf('clustered%.6d/', ind)];
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

%% Show all the partitioned clouds for one extracted door

ind = 3;
screenshot_folder = [picked_folder sprintf('screens%.6d/', ind)];
mkdir(screenshot_folder)

m = length(indices{ind});
for i = 1:4:m
    fileind = indices{ind}(i) - 1;
    index = [picked_folder sprintf('indices%.6d.txt', fileind)]
    pcdfile = [picked_folder sprintf('cloud%.6d.pcd', fileind)]
    screenshot = [screenshot_folder sprintf('graph%.6d_%.3d.png', fileind, i)]
    display_graph(pcdfile, index, screenshot, node_indices{ind}(:, i), length(G{fileind+1}.nodelabels));
end

%% Convert the graphs into dot files

for i = 1:3
    filename = [picked_folder 'area' sprintf('%.6d', i-1) '.dot'];
    mat2dot(doors{i}, filename);
end

%% Convert the dot files to images, show the graphs

for i = 1:3
    i
    filename = [picked_folder 'area' sprintf('%.6d', i-1) '.dot'];
    system(['dot -Teps ' filename ' > ' picked_folder 'area' sprintf('%.6d', i-1) '.eps']);
    system(['gvfs-open ' picked_folder 'area' sprintf('%.6d', i-1) '.eps']);
    pause
end