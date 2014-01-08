function convert_graphs(graph_folder, filename)

G = {};

listing = dir([graph_folder 'graph*.dot']);
l = length(listing);
for i = 1:l
    di = [graph_folder listing(i).name];
    G{i} = dot2mat(di);
end

filename = [graph_folder filename];
save(filename, 'G')

end