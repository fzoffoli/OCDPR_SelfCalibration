clear all
close all
clc

addpath('../../config')
addpath('../../data/workspace_files')
addpath('../../libs/cdpr_model')
addpath('../../libs/export_utilities')
addpath('../../libs/numeric')
addpath('../../libs/orientation_geometry')
addpath('../../libs/under_actuated')
addpath('../../libs/over_actuated')
folder = '../../data';
load data1_engraver.mat

t = dataT(:,1);
l = dataT(:,4:7);
c_mode = dataT(:,8:11);
T = dataT(:,12:15);
ind = zeros(1,4);
for j=1:4
   for i=1:length(t)
      if (c_mode(i,j) ~= c_mode(1,j)) 
         ind(1,j) = i;  
         break
      end
   end
end
for j=1:4
    if (ind(j) == 0)
        for i=1:length(t)
            if c_mode(i,j) == 2
                set_point(i,j) = T(i,j) - T(1,j); 
            else
                set_point(i,j) = l(i,j) - l(1,j);
            end
       end
    else
    for i=1:length(t)
        if (i<ind(j))
            if c_mode(i,j) == 2
                set_point(i,j) = T(i,j) - T(1,j); 
            else
                set_point(i,j) = l(i,j) - l(1,j);
            end
        else
            if c_mode(i,j) == 2
                set_point(i,j) = T(i,j) - T(ind(j),j); 
            else
                set_point(i,j) = l(i,j) - l(ind(j),j);
            end
        end
    end
    end
end



motors_id = [0 1 2 3];
%data = t';
motors_id_str = num2str(motors_id);
name = "prova_over";
filepath = strcat(name,".txt");
N = 4;
fid = fopen(filepath, 'w');
% 0 cable length
% 1 motor counts
% 2 motor speed 
% 3 motor torque
traj_type = 1;


fprintf(fid, strcat('%d', repmat(' %d', 1, N), '\n'), [traj_type, motors_id]);
fprintf(fid, strcat('%f', repmat(' %d', 1, N), repmat(' %f', 1, N), '\n'), [t c_mode set_point]');
fclose(fid);
fprintf("File saved in %s\n", filepath)