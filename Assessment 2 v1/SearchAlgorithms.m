clear all
clc

%% Resoluion, Amplitude and Time period
resolution = 1000;
amplitude = 100;
wavelength = 200;

%% x and y boundaries
x_min = 0;
x_max = 500;
y_min = -150;
y_max = 150;

%% Get x values
x = x_min:(x_max-x_min)/resolution:x_max;

%% Get y values for sinusoidal
y = amplitude * sin(x*((2*pi)/wavelength));

% Plot x vs y for sinusoidal
figure
plot(x,y)
hold on

%% Get y values for square wave
y = [];
y(end+1) = 0;
for x_i = x
    if x_i ~= 0
        if mod(x_i, wavelength) < wavelength/2
            y(end+1) = amplitude;
        else
            y(end+1) = -amplitude;
        end
    end
end

% Plot values for the square wave
plot(x, y)

%% Get x and y values for random search
x = (x_max-x_min)*rand(resolution, 1)+x_min;
y = (y_max-y_min)*rand(resolution, 1)+y_min;

x(1) = 0;
y(1) = 0;

% Plot values for the random search
figure
plot(x, y)