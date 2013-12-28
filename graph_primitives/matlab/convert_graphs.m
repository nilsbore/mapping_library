function convert_graphs()

G = {};

graph_folder = '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/';
listing = dir([graph_folder 'graph*.dot']);
l = length(listing);
for i = 1:l
    di = [graph_folder listing(i).name];
    G{i} = dot2mat(di);
end

filename = [graph_folder 'matgraphs.mat'];
save(filename, 'G')

end