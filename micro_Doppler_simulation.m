%
radarpos = [0;0;0];
radarvel = [0;0;0];

tgtinitpos = [3;0;0];
tgtvel     = [0;0;0];
tgtmotion  = phased.Platform('InitialPosition',tgtinitpos,'Velocity',tgtvel);

Nblades   = 2;
bladeang  = (0:Nblades-1)*2*pi/Nblades;
bladelen  = 0.127; %Blade length in meters
Blade_velocity_RPM = 10000; %Blade velocity in Rotation per minute(RPM)
Blade_velocity_RPS = Blade_velocity_RPM/60; %Blade velocity in Rotation per second(RPS)
Blade_rate = Blade_velocity_RPS * 360; %Blade rate in degree per second(deg/s)

bladerate = deg2rad(Blade_rate);  % rps -> rad/sec

c  = 3e8;
fc = 2.4e9;
helicop = phased.RadarTarget('MeanRCS',[0.01 .001 .001 ],'PropagationSpeed',c,...
    'OperatingFrequency',fc,'Mode','Bistatic');

fs     = 1e6;
prf    = 2e4;
lambda = c/fc;

wav = phased.RectangularWaveform('SampleRate',fs,'PulseWidth',2e-6,'PRF',prf);
ura = phased.URA('Size',4,'ElementSpacing',lambda/2);
tx  = phased.Transmitter;
rx  = phased.ReceiverPreamp;
env = phased.FreeSpace('PropagationSpeed',c,'OperatingFrequency',fc,...
    'TwoWayPropagation',true,'SampleRate',fs);
txant = phased.Radiator('Sensor',ura,'PropagationSpeed',c,'OperatingFrequency',fc);
rxant = phased.Collector('Sensor',ura,'PropagationSpeed',c,'OperatingFrequency',fc);

NSampPerPulse = round(fs/prf);
Niter = 4e3;
y     = complex(zeros(NSampPerPulse,Niter));
rng(2018);
for m = 1:Niter
    % update helicopter motion
    t = (m-1)/prf;
    [scatterpos,scattervel,scatterang] = helicopmotion(t,tgtmotion,bladeang,bladelen,bladerate);

    % simulate echo
    x  = txant(tx(wav()),scatterang);                    % transmit
    xt = env(x,radarpos,scatterpos,radarvel,scattervel); % propagates to/from scatterers
    xt = helicop(xt);                                    % reflect
    xr = rx(rxant(xt,scatterang));                       % receive
    y(:,m) = sum(xr,2);                                  % beamform
end

mfcoeff = getMatchedFilter(wav);
mf  = phased.MatchedFilter('Coefficients',mfcoeff);
ymf = mf(y);
[~,ridx] = max(sum(abs(ymf),2)); % detection via peak finding along range
pspectrum(ymf(ridx,:),prf,'spectrogram')

function [scatterpos,scattervel,scatterang] = helicopmotion(...
    t,tgtmotion,BladeAng,ArmLength,BladeRate)

prf = 2e4;
radarpos = [0;0;0];
Nblades  = size(BladeAng,2);

[tgtpos,tgtvel] = tgtmotion(1/prf);

RotAng     = BladeRate*t;
scatterpos = [0 ArmLength*cos(RotAng+BladeAng);0 ArmLength*sin(RotAng+BladeAng);zeros(1,Nblades+1)]+tgtpos;
scattervel = [0 -BladeRate*ArmLength*sin(RotAng+BladeAng);...
    0 BladeRate*ArmLength*cos(RotAng+BladeAng);zeros(1,Nblades+1)]+tgtvel;

[~,scatterang] = rangeangle(scatterpos,radarpos);

end
