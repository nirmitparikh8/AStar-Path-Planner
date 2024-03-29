% Define a scenario with a specified update rate
scenario = robotScenario(UpdateRate=5);

% Define floor properties
floorColor = [0.3 0.3 0.3];

% Define wall properties
wallHeight = 1;
wallWidth = 0.25;
wallLength = 10;
wallColor = [1 1 0.8157];

% Add a floor to the scenario
addMesh(scenario, "Plane", Position=[5 5 0], Size=[10 10], Color=floorColor);

% Add walls to the scenario
wallPositions = [
    % [x, y, z] positions of walls
    wallWidth/2, wallLength/2, wallHeight/2;
    wallLength-wallWidth/2, wallLength/2, wallHeight/2;
    wallLength/2, wallLength-wallWidth/2, wallHeight/2;
    wallLength/2, wallWidth/2, wallHeight/2;
    wallLength/8, wallLength/3, wallHeight/2;
    wallLength/4, wallLength/3.8, wallHeight/2;
    wallLength/3.3, wallLength/6, wallHeight/2;
    6.55, 8, wallHeight/2;
    wallLength/4, wallLength/1.7, wallHeight/2;
    7.5, 3, wallHeight/2;
    8, wallLength/1.7, wallHeight/2;
];
wallSizes = [
    % [x, y, z] sizes of walls
    wallWidth, wallLength, wallHeight;
    wallWidth, wallLength, wallHeight;
    wallLength, wallWidth, wallHeight;
    wallLength, wallWidth, wallHeight;
    wallLength/4, wallWidth, wallHeight;
    wallWidth, wallLength/6, wallHeight;
    wallLength/4, wallWidth, wallHeight;
    wallLength/1.5, wallWidth, wallHeight;
    wallLength/2.0, wallWidth, wallHeight;
    wallLength/2, wallWidth, wallHeight;
    wallLength/3, wallWidth, wallHeight;
];
% Add walls to scenario
for i = 1:size(wallPositions, 1)
    addMesh(scenario, "Box", Position=wallPositions(i,:), Size=wallSizes(i,:), Color=wallColor, IsBinaryOccupied=true);
end

% Create a binary occupancy map from the scenario
map = binaryOccupancyMap(scenario, GridOriginInLocal=[-2 -2], MapSize=[15 15], MapHeightLimits=[0 3]);
inflate(map, 0.3);

% Define start and goal positions for the robot
startPosition = [ 1.5 2.5];
goalPosition = [8 9];

% Plan a path using the A* algorithm
astar = mobileRobotPRM(map, 4000);
astar.ConnectionDistance = 1;
waypoints = findpath(astar, startPosition, goalPosition);

% Define robot height and create a trajectory for the robot to follow
robotheight = 0.12;
numWayPoints = size(waypoints, 1);
firstInTime = 0;
lastInTime = firstInTime + (numWayPoints -1);
trajectory = waypointTrajectory(SampleRate= 100, TimeOfArrival=firstInTime:lastInTime, Waypoints=[waypoints, robotheight*ones(numWayPoints,1)], ReferenceFrame = "ENU");

% Load a robot model
robot = loadrobot("clearpathJackal");

% Create a robot platform and attach the robot model to it
platform = robotPlatform("Jackal", scenario, RigidBodyTree=robot, BaseTrajectory=trajectory);

% Visualize the scenario and trajectory
[ax,plotFrames] = show3D(scenario);
lightangle(-45,30)
view(60,50)

% Plot waypoints on the scenario
hold(ax, "on");
plot(ax, waypoints(:,1), waypoints(:,2), "-ms", LineWidth = 2, MarkerSize=4, MarkerEdgeColor="g",  MarkerFaceColor=[0.5,0.5,0.5]);
hold(ax,"off");

% Setup the scenario
setup(scenario);
r = rateControl(20);

% Initialize variables for tracking robot movement
robotStartmoving = false;

% Main loop for advancing the scenario and robot movement
while advance(scenario)
    % Update the visualization
    show3D(scenario, Parent=ax, FastUpdate=true);
    waitfor(r);

    % Read current pose of the robot
    currentPose = read(platform);

    % Check if the robot has started moving
    if ~any(isnan(currentPose)) 
        robotStartMoving = true; 
    end
    
    % Break the loop if the robot stops moving
    if any(isnan(currentPose)) && robotStartMoving 
        break; 
    end
end
