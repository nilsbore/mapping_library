function Gprim = cluster_edges(G)

A = [];
p = primitives;

n = length(G);
for i = 1:n
    G{i}.edgeangles
    if G{i}.edgeangles
        A = [A; G{i}.edgeangles];
    end
end

hist(A, 20);
A

end