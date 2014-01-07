function G = dot2mat(filename)

fid = fopen(filename);
G = {};
G.edges = uint32([]);
G.edgeangles = [];
G.nodelabels = uint32([]);
G.nodesizes = [];

pat1 = '(?<node>\d+)\[label="(?<primitive>\w+)"\]?.+\[shapesize="(?<size>\d*\.?\d*?)"\]?.+';
pat2 = '(?<from>\d+)--(?<to>\d+).+\[label="(?<angle>\d*\.?\d*?)"\]?.+';

p = primitives;

alpha = 0.2;

while true
   tline = fgetl(fid);
   if ~ischar(tline)
       break
   end
   ll = length(tline);
   if ll < 10
       continue
   end
   if ll > 30
       n = regexp(tline, pat1, 'names')
       G.nodelabels = [G.nodelabels; uint32(0)];
       G.nodelabels(end) = cellfind(p, n.primitive);
       G.nodesizes = [G.nodesizes; str2double(n.size)];
   else
       n = regexp(tline, pat2, 'names')
       angle = str2double(n.angle);
       if angle < alpha
           G.edges = [G.edges; uint32(zeros(1, 3))];
           G.edges(end, 1) = str2num(n.from) + 1;
           G.edges(end, 2) = str2num(n.to) + 1;
           G.edges(end, 3) = 1;
       elseif angle > pi/2 - alpha
           G.edges = [G.edges; uint32(zeros(1, 3))];
           G.edges(end, 1) = str2num(n.from) + 1;
           G.edges(end, 2) = str2num(n.to) + 1;
           G.edges(end, 3) = 2;
       end
       %G.edgeangles = [G.edgeangles; angle];
   end
   %tline
end

fclose(fid);

end