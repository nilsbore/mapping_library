function ind = cellfind(D, x)

m = size(D, 2);
for i = 1:m
    if D{i} == x
        ind = i;
        return
    end
end

ind = -1; % not found

end