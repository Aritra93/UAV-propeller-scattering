%
Blade_velocity_RPM = 5000; %Blade velocity in Rotation per minute(RPM)
Blade_velocity_RPS = Blade_velocity_RPM/60; %Blade velocity in Rotation per second(RPS)
Blade_rate = Blade_velocity_RPS * 360; %Blade rate in degree per second(deg/s)

% fc = 5e9; %Carrier frequency(Hz)
%tc = 1/fc; %Repetition timing(s)
% Let us assume there is a repetation pulse with a defined frequency.
%f = 2e4; %Repetition frequency for pulse
% Total_time_window = 1;% Time window in seconds
% t = 1/f;
% arr_size = Total_time_window/t;
% rotation_ang_arr = zeros(arr_size);

%
propeller_vel = Blade_rate; %deg/s
time_step = 360/propeller_vel;
t = 0:0.00005:1;

rotation_angle = propeller_vel*t;
rotation_angle = mod(rotation_angle,360);
histogram(rotation_angle)
% numel(find(rotation_angle==75))

p = platform;
p.FileName = 'propeller_4_10_repaired.stl';
p.Units = 'in';
p.TiltAxis = 'Z';
p.UseFileAsMesh = true;
% p.Tilt = rotation_angle;
f2 = 2.4e9;
sigma = zeros(numel(t)-1,1);
for i = 1:200
    p.Tilt = rotation_angle(i);
    sigma(i)= rcs(p,f2,0,43,'Solver','PO',...
              'EnableGPU', false,...
              'Polarization','VV');
end
figure
plot((1:200),sigma(1:200,1),'^-r')

ax = gca;
ax.YLim = [-70,-5];
title('RCS PO 5000 RPM')
xlabel('Time(ms)')
ylabel('Magnitude, dBsm')
grid on
legend('prop1','Location','best')