function G = dot2mat2(filename)

global anglehist

fid = fopen(filename);
G = {};
G.edges = uint32([]);
G.edgeangles = [];
G.nodelabels = uint32([]);
G.nodesizes = [];
G.nodedata = [];

%pat1 = '(?<node>\d+)\[label="(?<primitive>\w+)"\]?.+\[shapesize="(?<size>\d*\.?\d*?)"\]?.+\[shapedata="(?<data>[^"]*)"\]?.+';
pat1 = '(?<node>\d+)\[label="(?<primitive>\w+)"\].*\[shapesize="-?(?<size>\d*\.?\d*(e-)?\d*)"\].*\[shapedata="(?<data>[^"]*)"\].*;';
pat2 = '(?<from>\d+)--(?<to>\d+).+\[label="(?<angle>\d*\.?\d*?)"\]?.+\[shapedist="(?<dist>\d*\.?\d*(e-)?\d*)"\]?.+';

p = primitives;
planeind = cellfind(p, 'Plane');

alpha = 0.52;

while true
   tline = fgetl(fid);
   if strfind(tline, 'nan')
       disp 'Found a NaN!'
       tline
       break
   end
   if ~ischar(tline)
       break
   end
   ll = length(tline);
   if ll < 10
       continue
   end
   if ll > 90 %30
       n = regexp(tline, pat1, 'names');
       G.nodelabels = [G.nodelabels; uint32(0)];
       %n
       %p 
       G.nodelabels(end) = cellfind(p, n.primitive);
       G.nodesizes = [G.nodesizes; str2double(n.size)];
       G.nodedata = [G.nodedata; zeros(1, 13)];
       if strcmp(n.data, 'floor')
           G.nodedata(end, 1:4) = [0, 0, 1, 0];
       else
           pars = strread(n.data);
           G.nodedata(end, 1:length(pars)) = pars;
       end
   else
       k = findstr(tline, '[style="dashed"]');
       adjacent = isempty(k);
       n = regexp(tline, pat2, 'names');
       tline
       n
       angle = str2double(n.angle);
       from = str2num(n.from) + 1;
       to = str2num(n.to) + 1;
       areplanes = G.nodelabels(from) == planeind && G.nodelabels(to) == planeind;
       if areplanes
           G.edgeangles = [G.edgeangles; angle];
       end
       if adjacent && areplanes
           anglehist = [anglehist angle];
           if abs(angle - pi/2) < alpha
               G.edges = [G.edges; uint32(zeros(1, 3))];
               G.edges(end, 1) = from;
               G.edges(end, 2) = to;
               G.edges(end, 3) = 1;
           elseif abs(angle - pi) < alpha
               G.edges = [G.edges; uint32(zeros(1, 3))];
               G.edges(end, 1) = from;
               G.edges(end, 2) = to;
               G.edges(end, 3) = 2;
           elseif abs(angle - 3*pi/2) < alpha
               G.edges = [G.edges; uint32(zeros(1, 3))];
               G.edges(end, 1) = from;
               G.edges(end, 2) = to;
               G.edges(end, 3) = 3;
           else
               continue
           end
       else
           if abs(angle - 0) < alpha || abs(angle - pi) < alpha
               if areplanes % distant parallell planes
                   fromn = G.nodedata(from, 1:3)'; % from plane normal
                   fromd = G.nodedata(from, 4);
                   ton = G.nodedata(to, 1:3)'; % to plane normal
                   tod = G.nodedata(to, 4);
                   fromc = G.nodedata(from, 7:9)';
                   toc = G.nodedata(to, 7:9)';
%                    if fromd < 0 % 0*n_from + d_from < 0 (pointing away from camera)
%                        fromn = -fromn;
%                        fromd = -fromd;
%                    end
%                    if tod < 0 % 0*n_to + d_to < 0 (pointing away from camera)
%                        ton = -ton;
%                        tod = -tod;
%                    end
                   if fromn'*ton < 0 % different directions
                       if all(fromc == 0) || all(toc) == 0 % floor always facing
                           G.edges = [G.edges; uint32(zeros(1, 3))];
                           G.edges(end, 1) = from;
                           G.edges(end, 2) = to;
                           G.edges(end, 3) = 2;
                       elseif (ton'-fromn')*(fromc - toc) > 0 % facing each other
                           G.edges = [G.edges; uint32(zeros(1, 3))];
                           G.edges(end, 1) = from;
                           G.edges(end, 2) = to;
                           G.edges(end, 3) = 2;
                       else % pointing away from each other
                           continue
                           %G.edges = [G.edges; uint32(zeros(1, 3))];
                           %G.edges(end, 1) = from;
                           %G.edges(end, 2) = to;
                           %G.edges(end, 3) = 5;
                       end
                   else % same direction
                       if abs(-tod/ton(3)+fromd/fromn(3)) < 0.3 % distance of planes
                           G.edges = [G.edges; uint32(zeros(1, 3))];
                           G.edges(end, 1) = from;
                           G.edges(end, 2) = to;
                           G.edges(end, 3) = 3;
                       else
                           G.edges = [G.edges; uint32(zeros(1, 3))];
                           G.edges(end, 1) = from;
                           G.edges(end, 2) = to;
                           G.edges(end, 3) = 4;
                       end
                   end
               else
                   G.edges = [G.edges; uint32(zeros(1, 3))];
                   G.edges(end, 1) = from;
                   G.edges(end, 2) = to;
                   G.edges(end, 3) = 2;
               end
           elseif abs(angle - pi/2) < alpha
               G.edges = [G.edges; uint32(zeros(1, 3))];
               G.edges(end, 1) = from;
               G.edges(end, 2) = to;
               G.edges(end, 3) = 1;
           elseif false%abs(angle - 3*pi/2) % TRY
               G.edges = [G.edges; uint32(zeros(1, 3))];
               G.edges(end, 1) = from;
               G.edges(end, 2) = to;
               G.edges(end, 3) = 5;
           else
               continue
           end
       end
       if ~adjacent
           G.edges(end, 3) = G.edges(end, 3) + 3; % + 4
       end
   end
   %tline
end

fclose(fid);

end