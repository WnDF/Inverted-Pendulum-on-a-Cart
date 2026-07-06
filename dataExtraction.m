%% Data Extraction Script for Simulink

% Extract scope data
t = inverted_pendulum_scope_data.time;

u     = inverted_pendulum_scope_data.signals(1).values;
angle = inverted_pendulum_scope_data.signals(2).values;
pos   = inverted_pendulum_scope_data.signals(3).values;

% Saving signals as columns
t     = t(:);
u     = u(:);
angle = angle(:);
pos   = pos(:);

% Create data structures
expData = struct();

expData.time = t;
expData.input.u = u;
expData.output.position = pos;
expData.output.angle = angle;

expData.sampleTime = h;
expData.signalNames = {'input_u','position','angle'};
expData.description = 'Inverted pendulum experiment data from Simulink Scope';

% ID Matrices 
expData.y = [pos angle];   
expData.u = u;             

% Save the data
save('pendulumTestData','expData');
