function N = test_cliques(G, min_nodes)

n = length(G{1}.nodelabels);
A = false(n);
for i = 1:size(G{1}.edges, 1)
    inds = G{1}.edges(i, 1:2);
    A(inds(1), inds(2)) = true;
    A(inds(2), inds(1)) = true;
end

mc = maximalCliques(A);
mc
size(mc)
number = sum(mc, 1);
large = find(number >= min_nodes);
number = number(large);
mc = mc(:, large);

nodes = cell(1, length(large));
for i = 1:length(large)
    nodes{i} = find(mc(:, i));
end
%nodes{1} = nodes{2};
%nodes{length(large)+2} = nodes{length(large)+1};

counter = 1;
k = length(nodes);
while counter <= k
    for i = 1:counter-1
        inter = intersect(nodes{counter}, nodes{i});
        if length(inter) >= min_nodes
            nodes{i} = union(nodes{counter}, nodes{i});
            nodes(counter) = [];
            counter = 0;
            k = k-1;
            break;
        end
    end
    counter = counter + 1;
end

edges = G{1}.edges(:, 1:2);
nedges = size(edges, 1);

N = cell(1, length(nodes));
for i = 1:length(nodes)
    N{i}.nodelabels = G{1}.nodelabels(nodes{i}, :);
    N{i}.nodesizes = G{1}.nodesizes(nodes{i}, :);
    N{i}.nodedata = G{1}.nodedata(nodes{i}, :);
    N{i}.edges = [];
    N{i}.edgeangles = [];
    N{i}.indices = nodes{i};
    for j = 1:length(nodes{i})
        for k = 1:j-1
            edge1 = [nodes{i}(j) nodes{i}(k)];
            edge2 = [nodes{i}(k) nodes{i}(j)];
            inds = find(all(ones(nedges, 1)*edge1 == edges, 2) | ...
                all(ones(nedges, 1)*edge2 == edges, 2));
            if isempty(inds)
                continue
            end
            if length(inds) > 1
                disp 'Something strange is happening, more than 1 of same edge...'
            end
            N{i}.edges = [N{i}.edges; j k G{1}.edges(inds(1), 3)];
            N{i}.edgeangles = [N{i}.edgeangles; G{1}.edgeangles(inds(1))];
%             for l = 1:size(G{1}.edges)
%                 inter = intersection(G{1}.edges(l, 1:2), edge);
%                 if length(inter) == 2
%                     
%                 end
%             end
        end
    end
end

end