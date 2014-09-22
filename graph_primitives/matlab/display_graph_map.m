function display_graph_map(pcd_file, index_files, screenshot_file, ...
    primitive_indices, nodes)

viewer = 'optirun ../bin/display_graph_map';
quote = '''';

ld_path = getenv('LD_LIBRARY_PATH');
setenv('LD_LIBRARY_PATH', '/opt/ros/groovy/lib');

orderings = '"';
for i = 1:length(primitive_indices)
    ordering = zeros(1, nodes{i});
    for j = 1:length(primitive_indices{i})
        ordering(primitive_indices{i}(j)) = j;
    end
    os = '';
    for j = 1:nodes{i}
        os = [os ' ' num2str(ordering(j))];
    end
    if i == 1
        orderings = [orderings os];
    else
        orderings = [orderings ', ' os];
    end
end
orderings = [orderings '"']
index_files
pcd_file

status = system([viewer ' "' pcd_file '" ' index_files ' ' orderings ' "' screenshot_file '"'], '-echo');

setenv('LD_LIBRARY_PATH', ld_path);

end