function convert_positions(data_folder, graph_folder, filename)

listing = dir([data_folder 'position*.txt']);
l = length(listing);
positions = zeros(l, 7);

for i = 1:l
    fid = fopen([data_folder listing(i).name]);
    tline = fgetl(fid);
    if ~ischar(tline)
        disp(['Could not read file ' listing(i).name])
        break
    end
    vals = strread(tline);
    if length(vals) ~= 7
        disp(['Not valid position in ' listing(i).name])
        break
    end
    positions(i, :) = vals;
    fclose(fid);
end

filename = [graph_folder filename];
save(filename, 'positions')

end