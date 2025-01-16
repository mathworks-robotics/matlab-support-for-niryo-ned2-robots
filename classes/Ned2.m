classdef Ned2 < handle
    % Copyright 2024-2025 The MathWorks, Inc.
    properties (SetAccess = private)
        RigidBodyTree;
        CmdAction;
        ArmCmdAction;
        GripAction;
        CamTopic;
        JointNames;
        IK;
        Weight;
        UseRos logical = true;
        JointStates;
        State
        HomeConfiguration = [0 0 0 0 0 0];
        IdleConfiguration = [-0.0038    0.6100   -1.3400   -0.0183   -0.3667    0.0032];
        sampleRate = 20;
        GripperStateSub;
        GripperState logical = false;
    end
    properties (SetAccess = public)
        plotTraj;
        robotAxes;
        recordTraj;
        RecordedPoses;
    end
    properties (SetObservable = true)
        CurrentConfiguration = [0 0 0 0 0 0];
        GripperConfiguration = [0 0];
    end

    methods
        function ned = Ned2(ip)
            arguments
                ip string = ""
            end
            if ip == ""
                error("Fill with the IP Address of Ned2")
            end
            ip = string(ip);
            ned.UseRos = logical(ip ~= "");
            if ned.UseRos && ~ros.internal.Global.isNodeActive
                ipaddress = strcat("http://",ip,":11311");
                setenv('ROS_MASTER_URI',ipaddress);
                rosinit(ipaddress);
            end
            if ~exist('niryo_ned2_gripper1_n_camera.mat','file')
                rbt = importrobot("niryo_ned2_gripper1_n_camera.urdf.xacro");
                save(fullfile(fileparts(which("niryo_ned2_gripper1_n_camera.urdf.xacro")),'niryo_ned2_gripper1_n_camera.mat'),'rbt');
            else
                S = load('niryo_ned2_gripper1_n_camera.mat');
                rbt = S.rbt;
            end
            ned.RigidBodyTree = rbt;
            ned.JointNames = {ned.RigidBodyTree.homeConfiguration.JointName}';
            ned.RigidBodyTree.DataFormat = 'row';

            % Add tool:
            gripper = robotics.RigidBody("end_effector");
            T = getTransform(ned.RigidBodyTree,[ned.CurrentConfiguration 0 0],'tool_link','base_link');
            S = se3(T(1:3,1:3)',[0,0,0.085]);
            setFixedTransform(gripper.Joint, S.tform);
            addBody(ned.RigidBodyTree, gripper, "tool_link");

            ned.IK = inverseKinematics("RigidBodyTree", ned.RigidBodyTree);

            ned.Weight = [1 1 1 1 1 1];

            if ned.UseRos
                ned.CmdAction  = rosactionclient("/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory");
                ned.GripAction.ActivationFcn = [];
                ned.CmdAction.FeedbackFcn = [];
                ned.CmdAction.ResultFcn = [];
                ned.CmdAction.ActivationFcn = [];

                ned.GripAction = rosactionclient("/niryo_robot_tools_commander/action_server");
                ned.GripAction.ActivationFcn = [];
                ned.GripAction.FeedbackFcn = [];
                ned.GripAction.ResultFcn = [];
                ned.CamTopic   = rossubscriber("/niryo_robot_vision/compressed_video_stream","DataFormat","struct");

                ned.JointStates = rossubscriber("/joint_states");
                ned.CurrentConfiguration = receive(ned.JointStates).Position';
                ned.JointStates.NewMessageFcn = @ned.JointStatesFcn;

                ned.GripperStateSub = rossubscriber("/niryo_robot_hardware/tools/motor");
                ned.GripperState = receive(ned.GripperStateSub).State == 17;
                ned.GripperStateSub.NewMessageFcn = @ned.GripperStateFcn;
            end
        end

        function GripperStateFcn(ned,~,msg)
            ned.GripperState = msg.State == 17;
            ned.GripperConfiguration = [-0.01 -0.01]*ned.GripperState;
        end

        function JointStatesFcn(ned,~,msg)
            ned.CurrentConfiguration = msg.Position';

            if ned.isRobotShown()
                ned.show();
            end

            if ned.recordTraj
                ned.RecordedPoses = [ned.RecordedPoses; ned.CurrentConfiguration(1:6)];
            end

        end

        function goHome(ned)
            CmdMsg = rosmessage(ned.CmdAction);
            CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            CmdPoint.Positions = ned.HomeConfiguration(1:6);
            CmdPoint.Velocities = zeros(1,6);
            CmdPoint.Accelerations = zeros(1,6);
            CmdPoint.TimeFromStart = ros.msg.Duration(3);
            CmdMsg.Trajectory.Header.Stamp = rostime("now") + rosduration(0.05);
            CmdMsg.Trajectory.JointNames = ned.JointNames(1:6);
            CmdMsg.Trajectory.Points = CmdPoint;
            
            ned.CmdAction.sendGoalAndWait(CmdMsg);
        end
        
        function openGripper(ned)
            ToolAction = rosmessage(ned.GripAction);
            ToolAction.Cmd.CmdType = ToolAction.Cmd.OPENGRIPPER;
            ToolAction.Cmd.HoldTorquePercentage = 100;
            ToolAction.Cmd.MaxTorquePercentage = 100;
            ToolAction.Cmd.ToolId = 11;

            ned.GripAction.sendGoalAndWait(ToolAction,3);
        end

        function closeGripper(ned)
            ToolAction = rosmessage(ned.GripAction);
            ToolAction.Cmd.CmdType = ToolAction.Cmd.CLOSEGRIPPER;
            ToolAction.Cmd.HoldTorquePercentage = 100;
            ToolAction.Cmd.MaxTorquePercentage = 100;
            ToolAction.Cmd.ToolId = 11;

            ned.GripAction.sendGoalAndWait(ToolAction,3);
        end

        function PlanTrajectory(ned, waypoints, duration, method)
            arguments
                ned
                waypoints = []
                duration = 3
                method = 'trapveltraj';
            end
            tvec = 0:1/ned.sampleRate:duration;
            numSamples = length(tvec);
            waypoints = waypoints';
            switch method
                case 'trapveltraj'
                    [q, qd, qdd] = trapveltraj(waypoints,numSamples);
                case 'minjerkpolytraj'
                    [q, qd, qdd] = minjerkpolytraj(waypoints,linspace(0,duration,size(waypoints,2)),numSamples);
                otherwise
                    disp('using trapveltraj by default')
                    [q, qd, qdd] = trapveltraj(waypoints,numSamples);
            end
            
            ned.SetPose(q', qd', qdd',tvec);
            
        end

        function SetPose(ned, Pose, Velocities, Accelerations, duration)
            arguments
                ned;
                Pose;
                Velocities = zeros(1,6);
                Accelerations = zeros(1,6);
                duration = 3
            end

            CmdMsg = rosmessage(ned.CmdAction);
            CmdPoints = CmdMsg.Trajectory.Points;

            for i=1:size(Pose,1)
            
                CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
                CmdPoint.Positions = [Pose(i,1:6)];
                CmdPoint.Velocities = [Velocities(i,1:6)];
                CmdPoint.Accelerations = [Accelerations(i,1:6)];
                CmdPoint.TimeFromStart = ros.msg.Duration(duration(i));
                CmdPoint.Effort = [zeros(1,6)];

                CmdPoints = [CmdPoints; CmdPoint];
            end

            CmdMsg.Trajectory.Header.Stamp = rostime("now");
            CmdMsg.Trajectory.JointNames = ned.JointNames(1:6);
            CmdMsg.Trajectory.Points = CmdPoints;
            
            ned.CmdAction.sendGoalAndWait(CmdMsg);
        end

        function Move(ned, Position, RPY, duration)
            arguments
                ned
                Position (1,3) double
                RPY (1,3) double
                duration = 3;
            end
            
            se = se3(flip(RPY),"eul","ZYX", Position); 
            configSoln = ned.IK("end_effector", se.tform, ned.Weight, [ned.CurrentConfiguration 0 0]);
            
            CmdMsg = rosmessage(ned.CmdAction);
            
            CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            CmdPoint.Positions = [configSoln(1:6)]';
            CmdPoint.Velocities = zeros(1,6);
            CmdPoint.Accelerations = zeros(1,6);
            CmdPoint.TimeFromStart = ros.msg.Duration(duration);

            CmdMsg.Trajectory.Header.Stamp = rostime("now") + rosduration(0.05);
            CmdMsg.Trajectory.JointNames = ned.JointNames(1:6);
            CmdMsg.Trajectory.Points = CmdPoint;
           
            ned.CmdAction.sendGoalAndWait(CmdMsg);
        end

        function image = snapshot(ned)
            image = [];
            try
                image = rosReadImage(ned.CamTopic.receive(2));
            catch
                warning("Camera not available");
            end
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

        function JointCalibration(ned, args)
            arguments
                ned
                args.ForceCalibration = false;
            end

            if args.ForceCalibration
                req = rossvcclient('/niryo_robot/joints_interface/request_new_calibration');
                call(req);
            end
            calService = rossvcclient('/niryo_robot/joints_interface/calibrate_motors');
            calMsg = rosmessage(calService);
            calMsg.Value = 1;
            res = call(calService,calMsg);
            disp(res.Message);
        end

        function val = isRobotShown(ned)
            val = ~isempty(ned.robotAxes) && isvalid(ned.robotAxes);
        end

        function goIdle(ned)
            CmdMsg = rosmessage(ned.CmdAction);
            CmdPoint = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            CmdPoint.Positions = ned.IdleConfiguration(1:6);
            CmdPoint.Velocities = zeros(1,6);
            CmdPoint.Accelerations = zeros(1,6);
            CmdPoint.TimeFromStart = ros.msg.Duration(3);
            CmdMsg.Trajectory.Header.Stamp = rostime("now") + rosduration(0.05);
            CmdMsg.Trajectory.JointNames = ned.JointNames(1:6);
            CmdMsg.Trajectory.Points = CmdPoint;
            
            ned.CmdAction.sendGoalAndWait(CmdMsg);
        end
    end
end