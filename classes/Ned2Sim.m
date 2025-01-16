classdef Ned2Sim < handle
    % Copyright 2024-2025 The MathWorks, Inc.
    properties (SetAccess = private)
        RigidBodyTree;
        IK;
        Weight;
        JointStates;
        State
        robotAxes = [];
        sampleRate = 20;
        HomeConfiguration;
    end
    properties (SetAccess = public)
        plotTraj = false;
        Gripper = false;
    end
    properties (SetObservable = true)
        CurrentConfiguration = [0 0 0 0 0 0];
        GripperConfiguration = [0 0];
    end
    methods
        function ned = Ned2Sim(args)
            arguments
                args.Tool = 'Gripper'
            end
            
            if ~exist('niryo_ned2_gripper1_n_camera.mat','file')
                rbt = importrobot("niryo_ned2_gripper1_n_camera.urdf.xacro");
                save(fullfile(fileparts(which("niryo_ned2_gripper1_n_camera.urdf.xacro")),'niryo_ned2_gripper1_n_camera.mat'),'rbt');
            else
                S = load('niryo_ned2_gripper1_n_camera.mat');
                rbt = S.rbt;
            end
            ned.RigidBodyTree = rbt;
            ned.RigidBodyTree.DataFormat = 'row';

            ned.HomeConfiguration = [zeros(1,6)];
            ned.CurrentConfiguration = ned.HomeConfiguration;
            
            % Add tool:
            if strcmp(args.Tool, 'Gripper')
                gripper = robotics.RigidBody("end_effector");
                T = getTransform(ned.RigidBodyTree,[ned.CurrentConfiguration 0 0],'tool_link','base_link');
                S = se3(T(1:3,1:3)',[0,0,0.085]);
                setFixedTransform(gripper.Joint, S.tform);
                addBody(ned.RigidBodyTree, gripper, "tool_link");
            end

            ned.IK = inverseKinematics("RigidBodyTree", ned.RigidBodyTree);
            ned.IK.SolverParameters.AllowRandomRestart = false;
            addlistener(ned,'CurrentConfiguration','PostSet',@ned.configurationChanged);
            addlistener(ned,'GripperConfiguration','PostSet',@ned.configurationChanged);
            ned.Weight = [1 1 1 1 1 1];
        end

        function configurationChanged(ned,~,~)
            if ned.isRobotShown()
                ned.show();
            end
        end

        function goHome(ned, duration)
            arguments
                ned
                duration = 3
            end
            ned.PlanTrajectory([ned.CurrentConfiguration;ned.HomeConfiguration],duration);
        end

        function PlanTrajectory(ned, waypoints, duration, method)
            arguments
                ned
                waypoints = [ned.CurrentConfiguration ned.GripperConfiguration;
                             ned.RigidBodyTree.randomConfiguration];
                duration = 3
                method = 'trapveltraj';
            end
            tvec = 0:1/ned.sampleRate:duration;
            numSamples = length(tvec);
            waypoints = waypoints';
            switch method
                case 'trapveltraj'
                    [q, ~] = trapveltraj(waypoints,numSamples);
                case 'minjerkpolytraj'
                    [q, ~] = minjerkpolytraj(waypoints,linspace(0,duration,size(waypoints,2)),numSamples);
                otherwise
                    disp('using trapveltraj by default')
                    [q, ~] = trapveltraj(waypoints,numSamples);
            end
            rc = rateControl(ned.sampleRate);
            
            for idx = 1:numSamples
                ned.CurrentConfiguration = q(1:6,idx)';
                waitfor(rc);
            end
        end

        function Move(ned, Position, RPY, duration)
            arguments
                ned
                Position (1,3) double
                RPY (1,3) double
                duration = 3;
            end
           
            configSoln = zeros(size(Position,1),8);
            curConf = [ned.CurrentConfiguration ned.GripperConfiguration];
            for i = 1:size(Position,1)
                se = se3(flip(RPY),"eul","ZYX", Position(i,:));
                configSoln(i,:) = ned.IK("end_effector", se.tform, ned.Weight, curConf);
                curConf = [configSoln(i,:) ned.GripperConfiguration];
            end

            ned.PlanTrajectory([ned.CurrentConfiguration ned.GripperConfiguration;configSoln],duration);

        end

        function SetPose(ned, Pose)
            arguments
                ned
                Pose
            end
            
            ned.CurrentConfiguration = Pose;
        end

        function show(ned,config)
            arguments
                ned
                config = [ned.CurrentConfiguration ned.GripperConfiguration];
            end
            if ~ned.isRobotShown()
                ned.clearFigure;
            else
                show(ned.RigidBodyTree, config, 'PreservePlot', false, 'FastUpdate', true,'Frames','off', 'Parent',ned.robotAxes);
            end
            if ned.plotTraj
                eepos = ned.RigidBodyTree.getTransform(config, 'end_effector','base_link');
                hold(ned.robotAxes,'on');
                plot3(eepos(1,4),eepos(2,4),eepos(3,4),'o','MarkerSize',3,'Color','r','MarkerFaceColor','r','Parent',ned.robotAxes,'LineJoin','miter')
                hold(ned.robotAxes,'off');
            end
            drawnow;
        end

        function clearFigure(ned,config)
            arguments
                ned
                config = [ned.CurrentConfiguration ned.GripperConfiguration];
            end
            if ned.isRobotShown
                cla(ned.robotAxes);
                show(ned.RigidBodyTree, config, 'PreservePlot', false, 'FastUpdate', true,'Frames','off','Parent',ned.robotAxes);
            else
                ned.robotAxes = show(ned.RigidBodyTree, config, 'PreservePlot', false, 'FastUpdate', true,'Frames','off');
            end
            set(ned.robotAxes.Parent, 'Windowstyle', 'docked');
            ned.robotAxes.Parent.Internal = 1;
            xlim(ned.robotAxes,[-0.5 0.5])
            ylim(ned.robotAxes,[-0.5 0.5])
            zlim(ned.robotAxes,[-0.1 0.8])
        end

        function val = isRobotShown(ned)
            val = ~isempty(ned.robotAxes) && isvalid(ned.robotAxes);
        end

        function openGripper(ned)
            arguments
                ned
            end
            ned.GripperConfiguration = [0 0];
        end

        function closeGripper(ned)
            arguments
                ned
            end
            ned.GripperConfiguration = [-0.01 -0.01];
        end
    end
end