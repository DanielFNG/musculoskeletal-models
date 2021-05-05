%% Here I'm going to run some PD controllers to:
%
% 1) Get the model from the start state to Andreas' start state.
% 4) Simulate the controls needed to swing up to the inverted state and
% stop.

%% Setup

% Get OpenSim libraries
import org.opensim.modeling.*

% Simulation parameters
start_time = 0;
total_time = 3;
step = 0.001;
timesteps = start_time:step:total_time;

% Kinematics goals
actuators = {'hip_motor_r', 'knee_motor_r', 'ankle_motor_r', ...
    'hip_motor_l', 'knee_motor_l', 'ankle_motor_l'};
joints = {'XoR_hip_flexion_r', 'XoR_knee_angle_r', 'XoR_ankle_angle_r', ...
    'XoR_hip_flexion_l', 'XoR_knee_angle_l', 'XoR_ankle_angle_l'};
goal_1 = [-20 -20 30 20 -20 -30];
goal_2 = -goal_1;
n_actuators = length(actuators);

% PD controller gains
position_gain = 100;
velocity_gain = 0.5;
integral_gain = 80;

% Directory paths
root = 'C:\Users\danie\Documents\GitHub\musculoskeletal-models\XoR';
results = [root filesep 'Results'];

%% Part 1: Get model to start state

% Import the model
xor_path = [root filesep 'XoR-gcomp-test.osim'];
xor = Model(xor_path);

% Create the exoskeleton controller.
xor_controller = PrescribedController();

% Add an actuator for each motor.
for i = 1:n_actuators
    actuator = xor.updActuators.get(actuators{i});
    xor_controller.addActuator(actuator);
    xor_controller.prescribeControlForActuator(actuators{i}, Constant(0));
end

% Add the controller to the model.
xor.addController(xor_controller);

% Get reference to underlying computational system
state = xor.initSystem();

% Create a simulation manager
simulation = Manager(xor);
simulation.initialize(state);

% Create some vectors for PD calculations
e = zeros(1, n_actuators);
e_old = e;
de = e;
ie = e;

% Create a vector to save some simulation data
positions = zeros(length(timesteps) - 1, n_actuators);
torques = zeros(length(timesteps) - 1, n_actuators);
errors = zeros(length(timesteps) - 1, n_actuators);
pt = zeros(length(timesteps) - 1, n_actuators);
dt = zeros(length(timesteps) - 1, n_actuators);

% Generate trajectories which linearly approach the goal in 1.5s.
goal_trajectory = ones(length(timesteps) - 1, n_actuators);
for i = 1:n_actuators
    goal_trajectory(:, i) = goal_1(i)*pi/180;
    start_value = xor.getCoordinateSet().get(joints{i}).getValue(state);
    goal_trajectory(1:1500, i) = linspace(start_value, goal_1(i)*pi/180, 1500);
end

% Simulate the system
for i = 1:length(timesteps) - 1
    disp(i)
    
    % Compute control values at this state.import 
    for j = 1:n_actuators
        % Get current joint position
        p = xor.getCoordinateSet().get(joints{j}).getValue(state);
        
        % Compute error term
        e_old(j) = e(j);
        e(j) = goal_trajectory(i, j) - p;
        
        % Compute error derivative term
        if i > 1
            de(j) = (e(j) - e_old(j))/step;
        end
        
        % Compute integral term
        ie(j) = trapz(errors(1:i, j))*step;
        
        % Compute & apply controls
        pd = position_gain * e(j) + velocity_gain * de(j) + integral_gain * ie(j);
        %fprintf('%s\t%f\t%s\t%f\t%s\t%f\t\n', 'pos', e(j), 'vel', de(j), 'int', ie(j));
        xor_controller.prescribeControlForActuator(actuators{j}, Constant(pd));
        
        % Save record of applied torques
        positions(i, j) = p;
        torques(i, j) = pd;
        errors(i, j) = e(j);
        pt(i, j) = position_gain * e(j);
        dt(i, j) = velocity_gain * de(j);
    end
    
    % Simulate
    state = simulation.integrate(timesteps(i + 1));  % Need state reference
end

% Plot resultant data
figure;
plot(positions(:, 4));
hold on
plot(goal_trajectory(:, 4));
plot(torques(:, 1));
plot(pt(:, 1));
plot(dt(:, 1));
legend('p', 'desired', 'torques', 'pt', 'dt');

%% Part 2: Swing to inverted state

% Simulation parameters
start_time = total_time;
total_time = 6;
timesteps = start_time:step:total_time;

% Create some vectors for PD calculations
e = zeros(1, n_actuators);
e_old = e;
de = e;
ie = e;

% Create a vector to save some simulation data
positions = zeros(length(timesteps) - 1, n_actuators);
torques = zeros(length(timesteps) - 1, n_actuators);
errors = zeros(length(timesteps) - 1, n_actuators);
pt = zeros(length(timesteps) - 1, n_actuators);
dt = zeros(length(timesteps) - 1, n_actuators);

% Generate trajectories which linearly approach the goal in 2s.
goal_trajectory = ones(length(timesteps) - 1, n_actuators);
for i = 1:n_actuators
    goal_trajectory(:, i) = goal_2(i)*pi/180;
    start_value = xor.getCoordinateSet().get(joints{i}).getValue(state);
    goal_trajectory(1:2000, i) = linspace(start_value, goal_2(i)*pi/180, 2000);
end

% Simulate the system
for i = 1:length(timesteps) - 1
    disp(i)
    
    % Compute control values at this state.import 
    for j = 1:n_actuators
        % Get current joint position
        p = xor.getCoordinateSet().get(joints{j}).getValue(state);
        
        % Compute error term
        e_old(j) = e(j);
        e(j) = goal_trajectory(i, j) - p;
        
        % Compute error derivative term
        if i > 1
            de(j) = (e(j) - e_old(j))/step;
        end
        
        % Compute integral term
        ie(j) = trapz(errors(1:i, j))*step;
        
        % Compute & apply controls
        pd = position_gain * e(j) + velocity_gain * de(j) + integral_gain * ie(j);
        %fprintf('%s\t%f\t%s\t%f\t%s\t%f\t\n', 'pos', e(j), 'vel', de(j), 'int', ie(j));
        xor_controller.prescribeControlForActuator(actuators{j}, Constant(pd));
        
        % Save record of applied torques
        positions(i, j) = p;
        torques(i, j) = pd;
        errors(i, j) = e(j);
        pt(i, j) = position_gain * e(j);
        dt(i, j) = velocity_gain * de(j);
    end
    
    % Simulate
    state = simulation.integrate(timesteps(i + 1));  % Need state reference
end

%% Printing results to file

% Write results to file
writeStatesData(simulation, [results filesep 'P1']);

function writeStatesData(manager, savedir)
% Use manager to write state data to file, optionally create data object. 

    import org.opensim.modeling.STOFileAdapter
    states = manager.getStatesTable();
    stofile = STOFileAdapter();
    if ~isfolder(savedir)
        mkdir(savedir);
    end
    stofile.write(states, [savedir filesep 'states.sto']);
    
end