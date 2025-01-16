function p = createRBT()
%   Copyright 2024-2025 The MathWorks, Inc.
workingDir = fullfile(pwd,"ned_ros","niryo_robot_description");
xacro = dir(fullfile(workingDir,'urdf','*\*.xacro'));
pattern = '<xacro:include filename="([^"]*\/)[^\/]+\.xacro"\/>';
for f = 1:numel(xacro)
    text = fileread(fullfile(xacro(f).folder,xacro(f).name));
    [a,b] = regexp(text,pattern,'match','tokens');

    for i = 1:numel(a)
        text = replace(text,b{i},'');
    end
    writelines(text,fullfile(fileparts(xacro(1).folder),xacro(f).name))

end
p = fullfile(fileparts(xacro(1).folder));
