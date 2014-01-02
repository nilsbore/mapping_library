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
       G.edges = [G.edges; uint32(zeros(1, 3))];
       G.edges(end, 1) = str2num(n.from) + 1;
       G.edges(end, 2) = str2num(n.to) + 1;
       G.edges(end, 3) = str2num(n.angle);
       G.edgeangles = [G.edgeangles; str2double(n.angle)];
       G.edges
   end
   %tline
end

fclose(fid);

end