function convert_graphs(graph_folder, filename)

global anglehist

anglehist = [];

G = {};

listing = dir([graph_folder 'graph*.dot']);
l = length(listing);
for i = 1:l
    di = [graph_folder listing(i).name];
    G{i} = dot2mat(di);
end

for i = 1:l
    %G{i} = filter_sizes(G{i});
end

filename = [graph_folder filename];
save(filename, 'G')

hist(180/pi*anglehist, 50)

end