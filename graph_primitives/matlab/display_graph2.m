function status = display_graph2(pcdfile, index, screenshot, indices, nodes)

viewer = 'optirun ../bin/display_graph';

ld_path = getenv('LD_LIBRARY_PATH');
setenv('LD_LIBRARY_PATH', '/opt/ros/groovy/lib');

ordering = zeros(1, nodes);
for j = 1:length(indices)
    ordering(indices(j)) = j;
end
os = '"';
nodes
for j = 1:nodes
    os = [os ' ' num2str(ordering(j))];
end
os = [os '"'];
status = system([viewer ' "' pcdfile '" "' index '" ' os ' "' screenshot '"'], '-echo');

setenv('LD_LIBRARY_PATH', ld_path);

end