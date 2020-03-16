% Define Arduino serial port here
arduino_port='com4'; % For window
arduino_port='/dev/cu.usbmodem1411'; % For MacOs X
arduino_port='/dev/ttyACM1'; % For Linux

% Standard modes (must correspond to similar mode in Ardunio code)
OPEN_LOOP=0;
CLASSICAL_CONTROLLER = 1;%classical (PID family) controller
STATE_SPACE=2;
% Define controller mode to be sent to the Arduino in get_response
mode = OPEN_LOOP;
%-------------------------------------------------------------
Tsample=10E-3;downsample=1;
n_meas_per_period=3;
n_period=40;
n_period_before=10;
Time=[0:n_period-1]'*Tsample*downsample;
if ~exist('arduino','var')
   arduino=init_serial(n_meas_per_period,n_period,arduino_port);
   arduino.n_period_before=n_period_before;
   arduino.downsample=downsample;
end
% Set adapted 'high speed' baudrate
%arduino.BaudRateTX=115200;
%arduino.BaudRateRX=115200;
%arduino.BaudRateRX=9600;


%% Measure step response
w0=-70.0;
err = set_mode_param(arduino,mode,w0,[]);
input('Press enter to continue')
w=70.0;

Y=get_response(arduino,w);
theta_x = Y(:,1);
theta_y = Y(:,2);
W = Y(:,3);
%accel_x=Y(:,1);accel_y=Y(:,2);accel_z=Y(:,3);


