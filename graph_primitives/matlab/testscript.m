addpath emgm

for ind = 1:4;
    clusters = 2;
    %label = emgm(V{ind},clusters);

    sigma = 0.04;
    [muhat, sigmahat] = normfit(V{ind}');
    label = 1 + double(mvnpdf(V{ind}', muhat, sigmahat) > sigma);

    colors = 'rgbmy';
    vi = V{ind} - vim*ones(1, size(V{ind}, 2));
    [U, S, VV] = svd(vi');
    vi = VV'*vi;
    figure
    hold on
    for i = 1:clusters
        plot3(vi(1,label==i), vi(2,label==i), vi(3,label==i), ['*' colors(i)])
    end
    
    indices{ind} = indices{ind}(label == 2);
    node_indices{ind} = node_indices{ind}(:, label == 2);
end