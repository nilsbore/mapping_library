function mat2dot(G, filename)

filename
fid = fopen(filename, 'w+');
fprintf(fid, 'graph G {\n');
p = primitives;
shapes = {'box', 'ellipse', 'ellipse'};

colors = {'#ff0000', '#00ff00', '#0000ff', '#ff00ff', '#ffff00', '#00ffff'};

for i = 1:length(G.nodelabels)
    v = G.nodelabels(i);
    fprintf(fid, '%d[label="%s"][shape="%s"][style="filled"][fillcolor="%s"];\n', i-1, p{v}, shapes{v}, colors{i});
end

for i = 1:length(G.edges)
    v = G.edges(i, :);
    fprintf(fid, '%d--%d[label="%d"]', v(1)-1, v(2)-1, v(3));
    if v(3) > 3
       fprintf(fid, '[style="dashed"]'); 
    end
    fprintf(fid, ';\n');
end

fprintf(fid, '}');
fclose(fid);

end