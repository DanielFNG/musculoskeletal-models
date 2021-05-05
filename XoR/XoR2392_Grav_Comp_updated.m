%% Gravity Compensation using XoR

% TODO: 
% - Take into consideration the translation between the exoskeleton
%   joint where forces are to be applied and the 2392 joint (small offset).
%   Consider also the hip adduction offset (different 2392 hip adduction to
%   exo hip adduction). However, in real life we will probably not know
%   these offsets either. 
% - Include hip adduction in grav comp to make it 3D? Is it needed? The
%   effect will probably be small due to small angles and hip adduction will
%   probably not be measured either in reality


clear;clc

import org.opensim.modeling.*

%% Import files (ignore for now)

XOR2392_model_path = strcat('XoR_gait2392_locked.osim');
XOR_model_path = strcat('..\XoR_model\XoRv4_locked.osim');
gait2392_model_path = strcat('gait2392v4.osim');

results_XOR2392_folder_path = strcat(pwd, '\Results\XOR2392_Grav_Comp');
%results_XOR_folder_path = strcat('Results\XOR_Grav_Comp');
results_gait2392_folder_path = strcat(pwd, '\Results\gait2392_Grav_Comp');



% Build OpenSim object using OpenSimTrial function that includes model,
% marker positions, a path for creating a results folder and ground reaction forces
XOR2392_ost = OpenSimTrial(XOR2392_model_path, [], results_XOR2392_folder_path);
%XOR_ost = OpenSimTrial(XOR_model_path, results_XOR_folder_path);
gait2392_ost = OpenSimTrial(gait2392_model_path, [], results_gait2392_folder_path);
%% Obtain Body Kinematics data 
% Ideally this will be obtained through MATLAB, using OpenSimTrial.m
% In this case BK was ran through OpenSim Gui and saved

BK_Data = Data('Results\XOR2392_Grav_Comp\3DGaitModel2392_BodyKinematics_pos_global.sto');

%% Reproduce normal gait
total_time =10;
start_time = 0.0;
time_steps = start_time:0.01:total_time;

initial_position_XOR = [40 -20 0 -20 30 -30];
initial_position_gait2392 = [40 -20 0 -20 30 -30];



XOR2392_model = Model('XOR-gait2392-v4-weld-bushings-fixed-gravity-on.osim');
g = abs(XOR2392_model.getGravity().get(1));


%% Get masses, centre of mass for exo and link lengths
m_thigh_r = XOR2392_model.get_BodySet().get('XoR_thigh_r').getMass();
COM_thigh_r =osimVec3ToArray(XOR2392_model.get_BodySet().get('XoR_thigh_r').getMassCenter());
m_thigh_l = XOR2392_model.get_BodySet().get('XoR_thigh_l').getMass();
COM_thigh_l =osimVec3ToArray(XOR2392_model.get_BodySet().get('XoR_thigh_l').getMassCenter());

m_shank_r = XOR2392_model.get_BodySet().get('XoR_shank_r').getMass();
COM_shank_r =osimVec3ToArray(XOR2392_model.get_BodySet().get('XoR_shank_r').getMassCenter());
m_shank_l = XOR2392_model.get_BodySet().get('XoR_shank_l').getMass();
COM_shank_l =osimVec3ToArray(XOR2392_model.get_BodySet().get('XoR_shank_l').getMassCenter());

m_foot_r = XOR2392_model.get_BodySet().get('XoR_foot_r').getMass();
COM_foot_r =osimVec3ToArray(XOR2392_model.get_BodySet().get('XoR_foot_r').getMassCenter());
m_foot_l = XOR2392_model.get_BodySet().get('XoR_foot_l').getMass();
COM_foot_l =osimVec3ToArray(XOR2392_model.get_BodySet().get('XoR_foot_l').getMassCenter());

masses_exo = [m_thigh_r; m_thigh_l; m_shank_r; m_shank_l; m_foot_r; m_foot_l];
COMs_exo = [COM_thigh_r; COM_thigh_l; COM_shank_r; COM_shank_l; COM_foot_r; COM_foot_l];

length_thigh_offset = osimVec3ToArray(XOR2392_model.getJointSet().get('XoR_thigh_r_XoR_shank_r').get_frames(0).get_translation()); 
length_thigh = abs(length_thigh_offset(2));
length_shank_offset = osimVec3ToArray(XOR2392_model.getJointSet().get('XoR_shank_r_XoR_foot_r').get_frames(0).get_translation()); 
length_shank = abs(length_shank_offset(2));

%% Get masses, centre of mass for human and link lengths
m_femur_r = XOR2392_model.get_BodySet().get('femur_r').getMass();
COM_femur_r =osimVec3ToArray(XOR2392_model.get_BodySet().get('femur_r').getMassCenter());
m_femur_l = XOR2392_model.get_BodySet().get('femur_l').getMass();
COM_femur_l =osimVec3ToArray(XOR2392_model.get_BodySet().get('femur_l').getMassCenter());

m_tibia_r = XOR2392_model.get_BodySet().get('tibia_r').getMass();
COM_tibia_r =osimVec3ToArray(XOR2392_model.get_BodySet().get('tibia_r').getMassCenter());
m_tibia_l = XOR2392_model.get_BodySet().get('tibia_l').getMass();
COM_tibia_l =osimVec3ToArray(XOR2392_model.get_BodySet().get('tibia_l').getMassCenter());


% Mass of talus is ignored since it is placed at the pivot point and
% creates no moments
 m_foot2392_r = XOR2392_model.get_BodySet().get('calcn_r').getMass() + ...
               XOR2392_model.get_BodySet().get('toes_r').getMass();
m_foot2392_l = XOR2392_model.get_BodySet().get('calcn_l').getMass() + ...
               XOR2392_model.get_BodySet().get('toes_l').getMass();




COM_talus_r = osimVec3ToArray(XOR2392_model.get_BodySet().get('talus_r').getMassCenter());
COM_talus_l = osimVec3ToArray(XOR2392_model.get_BodySet().get('talus_l').getMassCenter());

talus_calcn_offset_r = osimVec3ToArray(XOR2392_model.getJointSet().get('subtalar_r').get_frames(0).get_translation());
calcn_toes_offset_r = osimVec3ToArray(XOR2392_model.getJointSet().get('mtp_r').get_frames(0).get_translation());
talus_calcn_offset_l = osimVec3ToArray(XOR2392_model.getJointSet().get('subtalar_l').get_frames(0).get_translation());
calcn_toes_offset_l = osimVec3ToArray(XOR2392_model.getJointSet().get('mtp_l').get_frames(0).get_translation());

COM_calcn_r_calcn_frame = osimVec3ToArray(XOR2392_model.get_BodySet().get('calcn_r').getMassCenter());
COM_calcn_r_talus_frame = COM_calcn_r_calcn_frame + talus_calcn_offset_r; 

COM_calcn_l_calcn_frame = osimVec3ToArray(XOR2392_model.get_BodySet().get('calcn_l').getMassCenter());
COM_calcn_l_talus_frame = COM_calcn_l_calcn_frame + talus_calcn_offset_l; 

COM_toes_r_toes_frame = osimVec3ToArray(XOR2392_model.get_BodySet().get('toes_r').getMassCenter());
COM_toes_r_talus_frame = talus_calcn_offset_r + calcn_toes_offset_r + COM_toes_r_toes_frame;

COM_toes_l_toes_frame = osimVec3ToArray(XOR2392_model.get_BodySet().get('toes_l').getMassCenter());
COM_toes_l_talus_frame = talus_calcn_offset_l + calcn_toes_offset_l + COM_toes_l_toes_frame;

COM_foot2392_r = (COM_talus_r*XOR2392_model.get_BodySet().get('talus_r').getMass() +...
                   COM_calcn_r_talus_frame*XOR2392_model.get_BodySet().get('calcn_r').getMass() + ...
                   COM_toes_r_talus_frame*XOR2392_model.get_BodySet().get('toes_r').getMass())/m_foot2392_r;
COM_foot2392_l = (COM_talus_l*XOR2392_model.get_BodySet().get('talus_l').getMass() +...
                   COM_calcn_l_talus_frame*XOR2392_model.get_BodySet().get('calcn_l').getMass() + ...
                   COM_toes_l_talus_frame*XOR2392_model.get_BodySet().get('toes_l').getMass())/m_foot2392_l;

masses_2392 = [m_femur_r; m_femur_l; m_tibia_r; m_tibia_l; m_foot2392_r; m_foot2392_l];
COMs_2392 = [COM_femur_r; COM_femur_l; COM_tibia_r; COM_tibia_l; COM_foot2392_r; COM_foot2392_l];


femur_tibia_offset = osimVec3ToArray(XOR2392_model.getJointSet().get('knee_r').get_frames(0).get_translation()); 
%length_femur = abs(femur_tibia_offset(2));
length_femur = 0.395; % value obtained from OpenSim GUI since the translational offset in osim file is zero


tibia_talus_offset = osimVec3ToArray(XOR2392_model.getJointSet().get('ankle_r').get_frames(0).get_translation());
length_tibia = abs(tibia_talus_offset(2));

%% Calculate angle offsets (where the model reaches equilibrium based on CoMs location)

dq_ankle_exo = atan(COMs_exo(5,1)/abs(COMs_exo(5,2)));
dq_knee_exo = atan(COMs_exo(3,1)/abs(COMs_exo(3,2)));
dq_hip_exo = atan(COMs_exo(1,1)/abs(COMs_exo(1,2)));

dq_ankle_2392 = atan(COMs_2392(5,1)/abs(COMs_2392(5,2)));
dq_knee_2392 = atan(COMs_2392(3,1)/abs(COMs_2392(3,2)));
dq_hip_2392 = atan(COMs_2392(1,1)/abs(COMs_2392(1,2)));

%% Labels
DOF_2392_labels = {'hip_flexion_r'; 'hip_flexion_l'; 'knee_angle_r';'knee_angle_l';'ankle_angle_r';'ankle_angle_l'};
DOF_XOR_labels = {'XoR_hip_flexion_r'; 'XoR_hip_flexion_l'; 'XoR_knee_angle_r';'XoR_knee_angle_l';'XoR_ankle_angle_r';'XoR_ankle_angle_l'};
actuator_labels = {'hip_motor_r'; 'hip_motor_l'; 'knee_motor_r'; 'knee_motor_l'; 'ankle_motor_r'; 'ankle_motor_l'};

%% Run Simulation
controller_set = cell(1, size(masses_exo,1));
actuator_set = cell(1, size(masses_exo,1));
function_set = cell(1, size(masses_exo,1));
for i = 1:size(masses_exo,1)
    controller_set{i} = PrescribedController();
    actuator_set{i} = XOR2392_model.updActuators.get(actuator_labels{i});
    controller_set{i}.addActuator(actuator_set{i});
    controller_set{i}.prescribeControlForActuator(actuator_labels{i}, Constant(0));
    XOR2392_model.addController(controller_set{i});
end

% Get reference to underlying computational system
state = XOR2392_model.initSystem();
coordinates = XOR2392_model.updCoordinateSet();
for i = 1:size(masses_exo,1)
    coordinates.get(DOF_2392_labels{i}).setValue(state, initial_position_gait2392(i)*pi/180);
    coordinates.get(DOF_XOR_labels{i}).setValue(state, initial_position_XOR(i)*pi/180);
end

XOR2392_model.equilibrateMuscles(state);

simulation = Manager(XOR2392_model);
state.setTime(start_time);
simulation.initialize(state);

factor = 0.096;
damping_factor = 0.2;
grav_comp = zeros(length(time_steps), length(DOF_2392_labels));
py_XOR = zeros(length(time_steps), length(DOF_2392_labels));
py_2392 = zeros(length(time_steps), length(DOF_2392_labels));

% Simulate
for i = 1:length(time_steps)-1
    % Compute control value at this state.
    for j = 1:length(DOF_2392_labels)
        if j>4
            py_ankle_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j}).getValue(state);
            pvy_ankle_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j}).getSpeedValue(state);
            py_knee_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j-2}).getValue(state);
            py_hip_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j-4}).getValue(state);
            py_ankle_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j}).getValue(state);
            py_knee_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j-2}).getValue(state);
            py_hip_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j-4}).getValue(state);
            pvy_ankle_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j}).getSpeedValue(state);
            
            py_XOR(i,j) = py_ankle_XOR;
            py_XOR(i,j-2) = py_knee_XOR;
            py_XOR(i,j-4) = py_hip_XOR;
            py_2392(i,j) = py_ankle_2392;
            py_2392(i,j-2) = py_knee_2392;
            py_2392(i,j-4) = py_hip_2392;
            
            grav_comp(i,j) = (g*masses_exo(j)*sqrt(abs(COMs_exo(j,2))^2+abs(COMs_exo(j,1))^2)*sin(py_ankle_XOR + py_knee_XOR + py_hip_XOR + dq_ankle_exo)) +...
                            (g*masses_2392(j)*sqrt(abs(COMs_2392(j,2))^2+abs(COMs_2392(j,1))^2)*sin(py_ankle_2392 + py_knee_2392 + py_hip_2392 + dq_ankle_2392)) -...
                            pvy_ankle_XOR*(g*(masses_exo(j)+masses_2392(j))*sqrt(abs(COMs_exo(j,2))^2+abs(COMs_exo(j,1))^2))*damping_factor*0.7; %+...
            controller_set{j}.prescribeControlForActuator(actuator_labels{j},Constant(grav_comp(i,j)*factor));
        elseif j<5 && j>2
            py_ankle_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j+2}).getValue(state);    
            py_knee_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j}).getValue(state);
            pvy_knee_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j}).getSpeedValue(state);
            py_hip_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j-2}).getValue(state);
            py_ankle_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j+2}).getValue(state);
            py_knee_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j}).getValue(state);
            py_hip_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j-2}).getValue(state);     

            grav_comp(i,j) = (g*masses_exo(j)*sqrt(abs(COMs_exo(j,2))^2+abs(COMs_exo(j,1))^2)*sin(py_knee_XOR + py_hip_XOR + dq_knee_exo) + g*masses_exo(j+2)*length_shank*sin(py_knee_XOR+py_hip_XOR) + g*masses_exo(j+2)*sqrt(abs(COMs_exo(j+2,2))^2+abs(COMs_exo(j+2,1))^2)*sin(py_ankle_XOR+py_knee_XOR+py_hip_XOR + dq_ankle_exo)) + ...
                            (g*masses_2392(j)*sqrt(abs(COMs_2392(j,2))^2+abs(COMs_2392(j,1))^2)*sin(py_knee_2392 + py_hip_2392 + dq_knee_2392) + g*masses_2392(j+2)*length_tibia*sin(py_knee_2392+py_hip_2392) + g*masses_2392(j+2)*sqrt(abs(COMs_2392(j+2,2))^2+abs(COMs_2392(j+2,1))^2)*sin(py_ankle_2392+py_knee_2392+py_hip_2392 + dq_ankle_2392)) +...
                             - pvy_knee_XOR*(g*(masses_exo(j)+masses_2392(j))*sqrt(abs(COMs_exo(j,2))^2+abs(COMs_exo(j,1))^2))*damping_factor; %+...
            controller_set{j}.prescribeControlForActuator(actuator_labels{j},Constant(grav_comp(i,j)*factor));
        elseif j<3
            py_ankle_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j+4}).getValue(state);    
            py_knee_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j+2}).getValue(state);
            py_hip_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j}).getValue(state);
            pvy_hip_XOR = XOR2392_model.getCoordinateSet().get(DOF_XOR_labels{j}).getSpeedValue(state);           
            py_ankle_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j+4}).getValue(state);
            py_knee_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j+2}).getValue(state);
            py_hip_2392 = XOR2392_model.getCoordinateSet().get(DOF_2392_labels{j}).getValue(state);     

            grav_comp(i,j) = (g*masses_exo(j)*sqrt(abs(COMs_exo(j,2))^2 +abs(COMs_exo(j,1))^2)*sin(py_hip_XOR + dq_hip_exo) + g*masses_exo(j+2)*((length_thigh*sin(py_hip_XOR))+(sqrt(abs(COMs_exo(j+2,1))^2+abs(COMs_exo(j+2,2))^2)*sin(py_knee_XOR+py_hip_XOR + dq_knee_exo))) + g*masses_exo(j+4)*((length_thigh*sin(py_hip_XOR))+ (length_shank*sin(py_knee_XOR+py_hip_XOR)) + (sqrt(abs(COMs_exo(j+4,1))^2+abs(COMs_exo(j+4,2))^2)*sin(py_ankle_XOR+py_knee_XOR+py_hip_XOR +dq_ankle_exo)))) +...
                            (g*masses_2392(j)*sqrt(abs(COMs_2392(j,2))^2 + abs(COMs_2392(j,1))^2)*sin(py_hip_2392 + dq_hip_2392) + g*masses_2392(j+2)*((length_femur*sin(py_hip_2392))+(sqrt(abs(COMs_2392(j+2,1))^2+abs(COMs_2392(j+2,2))^2)*sin(py_knee_2392+py_hip_2392 + dq_knee_2392))) + g*masses_2392(j+4)*((length_femur*sin(py_hip_2392))+ (length_tibia*sin(py_knee_2392+py_hip_2392)) + (sqrt(abs(COMs_2392(j+4,1))^2+abs(COMs_2392(j+4,2))^2)*sin(py_ankle_2392+py_knee_2392+py_hip_2392 +dq_ankle_2392)))) +...
                            - pvy_hip_XOR*(g*(masses_exo(j)+masses_2392(j))*sqrt(abs(COMs_exo(j,2))^2+abs(COMs_exo(j,1))^2))*damping_factor; %+...
            controller_set{j}.prescribeControlForActuator(actuator_labels{j},Constant(grav_comp(i,j)*factor));
        end
    end
    i
    state = simulation.integrate(time_steps(i+1));
end

disp('Saving File..')
writeStatesData(simulation, [pwd filesep 'grav-comp-results']);
beep;

% plot differences in exo joints vs human joint
%% Plot forces and joint angles

figure('Position', [50 50 1500 700])
subplot(3,2,1)
yyaxis left
plot(time_steps, [grav_comp(:,1)])
ylabel('Hip torque')
yyaxis right
plot(time_steps, [py_XOR(:,1) py_2392(:,1)])
title('Right Side')
ylabel('Angle')

subplot (3,2,2)
yyaxis left
plot(time_steps, [grav_comp(:,2)])
yyaxis right
plot(time_steps, [py_XOR(:,2) py_2392(:,2)])

title('Left Side')
subplot (3,2,3)
yyaxis left
plot(time_steps, [grav_comp(:,3)])
ylabel('Knee torque')
yyaxis right
plot(time_steps, [py_XOR(:,3) py_2392(:,3)])
ylabel('Angle')

subplot (3,2,4)
yyaxis left
plot(time_steps, [grav_comp(:,4)])
yyaxis right
plot(time_steps, [py_XOR(:,4) py_2392(:,4)])

subplot (3,2,5)
yyaxis left
plot(time_steps, [grav_comp(:,5)])
ylabel('Ankle torque')
yyaxis right
plot(time_steps, [py_XOR(:,5) py_2392(:,5)])
ylabel('Angle')

subplot (3,2,6)
yyaxis left
plot(time_steps, [grav_comp(:,6)])
yyaxis right
plot(time_steps, [py_XOR(:,6) py_2392(:,6)])
legend('Grav Comp','XOR_q','Human_q')

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

