%% Section 0: Setup

% Paths
root = 'C:\Users\danie\Documents\GitHub\musculoskeletal-models\XoR';
model_path = [root filesep 'XoR-gait2392-v4-weld.osim'];
results = [root filesep 'Results'];

% Import OpenSim libraries
import org.opensim.modeling.*

%% Section 1: Running a Simulation with Pre-Calculated Muscle Controls

% Simulation parameters
start_time = 0.53;
end_time = 2.49;

% Import the XoR model
xor = Model(model_path);

% Create a set of controllers for each muscle.
control_data = Data([root filesep 'subject01_walk1_controls.sto']);
timesteps = control_data.getTimesteps();
n_muscles = control_data.NCols - 1;
controller_set = cell(1, n_muscles);
actuator_set = cell(1, n_muscles);
function_set = cell(1, n_muscles);

for i = 1:n_muscles
    % Set up controller and actuator
    actuator = control_data.Labels{i + 1};
    controller_set{i} = PrescribedController();
    actuator_set{i} = xor.updActuators().get(actuator);
    controller_set{i}.addActuator(actuator_set{i});
    
    % Generate a piecewise constant control function.
    function_set{i} = PiecewiseConstantFunction();
    values = control_data.getColumn(actuator);
    for j = 1:control_data.NFrames
        function_set{i}.addPoint(timesteps(j), values(j));
    end
    
    % Assign function to controller
    controller_set{i}.prescribeControlForActuator(actuator, function_set{i});
    
    % Add to model
    xor.addController(controller_set{i});
end

% Get reference to underlying computational system
state = xor.initSystem();

% Initialise muscle states
xor.equilibrateMuscles(state);

% Create a simulation manager.
simulation = Manager(xor);
state.setTime(start_time);
simulation.initialize(state);

% Simulate the system for 2s
simulation.integrate(end_time);

% Write results to file
writeStatesData(simulation, [results filesep 'S1']);

%% Section 2: Running a Weakened Simulation

% Simulation parameters
start_time = 0.53;
end_time = 2.49;

% Import the XoR model
model_path = [root filesep 'XoR-gait2392-v4-weld-weak.osim'];
xor = Model(model_path);

% Create a set of controllers for each muscle.
control_data = Data([root filesep 'subject01_walk1_controls.sto']);
timesteps = control_data.getTimesteps();
n_muscles = control_data.NCols - 1;
controller_set = cell(1, n_muscles);
actuator_set = cell(1, n_muscles);
function_set = cell(1, n_muscles);

for i = 1:n_muscles
    % Set up controller and actuator
    actuator = control_data.Labels{i + 1};
    controller_set{i} = PrescribedController();
    actuator_set{i} = xor.updActuators().get(actuator);
    controller_set{i}.addActuator(actuator_set{i});
    
    % Generate a piecewise constant control function.
    function_set{i} = PiecewiseConstantFunction();
    values = control_data.getColumn(actuator);
    for j = 1:control_data.NFrames
        function_set{i}.addPoint(timesteps(j), values(j));
    end
    
    % Assign function to controller
    controller_set{i}.prescribeControlForActuator(actuator, function_set{i});
    
    % Add to model
    xor.addController(controller_set{i});
end

% Get reference to underlying computational system
state = xor.initSystem();

% Initialise muscle states
xor.equilibrateMuscles(state);

% Create a simulation manager.
simulation = Manager(xor);
state.setTime(start_time);
simulation.initialize(state);

% Simulate the system for 2s
simulation.integrate(end_time);

% Write results to file
writeStatesData(simulation, [results filesep 'S2']);

%% Section 3: Running an Assisted Simulation

% Simulation parameters
start_time = 0.53;
end_time = 2.49;

% Import the XoR model
model_path = [root filesep 'XoR-gait2392-v4-weld-weak.osim'];
xor = Model(model_path);

% Create a set of controllers for each muscle.
control_data = Data([root filesep 'subject01_walk1_controls.sto']);
timesteps = control_data.getTimesteps();
n_muscles = control_data.NCols - 1;
controller_set = cell(1, n_muscles);
actuator_set = cell(1, n_muscles);
function_set = cell(1, n_muscles);

for i = 1:n_muscles
    % Set up controller and actuator
    actuator = control_data.Labels{i + 1};
    controller_set{i} = PrescribedController();
    actuator_set{i} = xor.updActuators().get(actuator);
    controller_set{i}.addActuator(actuator_set{i});
    
    % Generate a piecewise constant control function.
    function_set{i} = PiecewiseConstantFunction();
    values = control_data.getColumn(actuator);
    for j = 1:control_data.NFrames
        function_set{i}.addPoint(timesteps(j), values(j));
    end
    
    % Assign function to controller
    controller_set{i}.prescribeControlForActuator(actuator, function_set{i});
    
    % Add to model
    xor.addController(controller_set{i});
end

% Create the XoR hip controller
xor_controller = PrescribedController();
xor_hip = xor.updActuators.get('hip_motor_r');
xor_controller.addActuator(xor_hip);
xor_controller.prescribeControlForActuator('hip_motor_r', Constant(0));
xor.addController(xor_controller);

% Get reference to underlying computational system
state = xor.initSystem();

% Initialise muscle states
xor.equilibrateMuscles(state);

% Create a simulation manager.
simulation = Manager(xor);
state.setTime(start_time);
simulation.initialize(state);

% Simulate the system for 2s
velocity_gain = 8;
position_gain = 20;
desired_data = Data([root filesep 'subject01_walk1_ik.mot']);
desired_timesteps = desired_data.getTimesteps();
desired_trajectory = desired_data.getColumn('hip_flexion_r');
for i = 3:length(desired_timesteps)
    py = xor.getCoordinateSet.get('hip_flexion_r').getValue(state);
    pvy = xor.getCoordinateSet.get('hip_flexion_r').getSpeedValue(state);
    pd = -velocity_gain * pvy - position_gain * (py - desired_trajectory(i)*pi/180);
    xor_controller.prescribeControlForActuator('hip_motor_r', Constant(pd));
    
    % Simulate
    state = simulation.integrate(desired_timesteps(i));
end

% Write results to file
writeStatesData(simulation, [results filesep 'S3']);

%% Helper Functions

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

