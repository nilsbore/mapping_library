function mat2dot(G, filename)

filename
fid = fopen(filename, 'w+');
fprintf(fid, 'graph G {\n');
p = primitives;
shapes = {'box', 'ellipse', 'ellipse'};

for i = 1:length(G.nodelabels)
    v = G.nodelabels(i);
    fprintf(fid, '%d[label="%s"][shape="%s"];\n', i-1, p{v}, shapes{v});
end

for i = 1:length(G.edges)
    v = G.edges(i, :);
    fprintf(fid, '%d--%d[label="%f"];\n', v(1)-1, v(2)-1, v(3));
end

fprintf(fid, '}');
fclose(fid);

end