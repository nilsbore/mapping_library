
addpath '/home/nbore/Installs/gboost-0.1.1/bin';
load '/home/nbore/Workspace/mapping_library/graph_primitives/graphs/matgraphs.mat'
[subg, count] = gspan(G, 5, [4 0]);