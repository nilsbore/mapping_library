function components = connected_components(X, threshold)

n = size(X, 2);
C = false(n);

for i = 1:n
    C(i, i) = true;
    for j = 1:i-1
        dist = norm(X(:, i) - X(:, j));
        C(i, j) = dist < threshold;
        C(j, i) = C(i, j);
    end
end

[p,q,r,s] = dmperm(C);

m = length(r) - 1;
components = cell(1, m);
for i = 1:m
    components{i} = p(r(i):r(i+1)-1);
end

end