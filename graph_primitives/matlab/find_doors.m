%% Gspan for finding the typical door

required = 8; % closed
%required = 4; % inside
%required = 5; % outside

min_nodes = 5;
[subg, count, GY, indices, node_indices] = gspan(G, required, [min_nodes 5]);
n = length(subg);

%% See how many of the respective doors are in the data

door_folder = '/home/nbore/Workspace/mapping_library/graph_primitives/matlab/doors/';
load('doors.mat')

doorpresent = false(3, length(G));
node_indices = {[], [], []};

maxedges = 5*(5-1)/2;
m = length(G);
for i = 1:m
    for j = 1:3
        [subgt, countt, GYt, indicest, node_indicest] = gspan({G{i} doors{j}}, 2, [5 5]);
        for k = 1:length(subgt)
            if length(subgt{k}.edges) == maxedges
                doorpresent(j, i) = true;
                I = find(indicest{k} == 1);
                node_indices{j} = [node_indices{j} node_indicest{k}(:, I(1))];
                continue;
            end
        end
    end
end

indices = {};
indices{1} = find(doorpresent(1, :));
indices{2} = find(doorpresent(2, :));
indices{3} = find(doorpresent(3, :));

%% Show all the partitioned clouds for one extracted door

ind = 3;
screenshot_folder = [door_folder sprintf('door%.6d/', ind)];
mkdir(screenshot_folder)

m = length(indices{ind});
for i = 1:4:m
    fileind = indices{ind}(i) - 1;
    index = [graph_folder sprintf('indices%.6d.txt', fileind)]
    pcdfile = [data_folder sprintf('cloud%.6d.pcd', fileind)]
    screenshot = [screenshot_folder sprintf('graph%.6d_%.3d.png', fileind, i)]
    display_graph(pcdfile, index, screenshot, node_indices{ind}(:, i), length(G{fileind+1}.nodelabels));
end

%% Convert the graphs into dot files

for i = 1:3
    filename = [door_folder 'door' sprintf('%.6d', i-1) '.dot'];
    mat2dot(doors{i}, filename);
end

%% Convert the dot files to images, show the graphs

for i = 1:3
    i
    filename = [door_folder 'door' sprintf('%.6d', i-1) '.dot'];
    system(['dot -Teps ' filename ' > ' door_folder 'door' sprintf('%.6d', i-1) '.eps']);
    system(['gvfs-open ' door_folder 'door' sprintf('%.6d', i-1) '.eps']);
    pause
end