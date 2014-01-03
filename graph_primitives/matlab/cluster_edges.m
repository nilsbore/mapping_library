function Gprim = cluster_edges(G)

X = cell(3, 3);

n = length(G);
for i = 1:n
    Gi = G{i};
    m = size(Gi.edges, 1);
    for j = 1:m
        ej = Gi.edges(j, :);
        p1 = Gi.nodelabels(ej(1));
        p2 = Gi.nodelabels(ej(2));
        sz1 = Gi.nodesizes(ej(1));
        sz2 = Gi.nodesizes(ej(2));
        if p1 > p2 || (p1 == p2 && sz1 > sz2)
            temp = p2;
            p2 = p1;
            p1 = temp;
            temp = sz2;
            sz2 = sz1;
            sz1 = temp;
        end
        X{p1, p2} = [X{p1, p2} [Gi.edgeangles(j); sz1; sz2]];
    end
end

p = primitives;
planei = cellfind(p, 'Plane');
cyli = cellfind(p, 'Cylinder');
spherei = cellfind(p, 'Sphere');

plot3(X{planei, cyli}(1, :), X{planei, cyli}(2, :), X{planei, cyli}(3, :), '*')
figure
plot3(X{planei, planei}(1, :), X{planei, planei}(2, :), X{planei, planei}(3, :), '*')
figure
plot3(X{cyli, cyli}(1, :), X{cyli, cyli}(2, :), X{cyli, cyli}(3, :), '*')
Gprim = [];

end