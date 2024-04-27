clear all
clc

%% Resoluion, Amplitude and Time period
resolution = 200;
amplitude = 100;
wavelength = 400;

%% x and y boundaries
x_min = 0;
x_max = 420;
y_min = -297/2;
y_max = 297/2;

%% Get x values
x = x_min:(x_max-x_min)/resolution:x_max;

figure
all_distances = [];
for wave_i = [200, 400]
    wavelength = wave_i;

    %% Get y values for sinusoidal
    y = amplitude * sin(x*((2*pi)/wavelength));

    %% Calculate the disance of the path
    distance = 0;
    for i = 2:length(x)
        distance = distance + sqrt((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2);
    end
    all_distances(end+1) = distance;

    fprintf('Distance for sinusoidal wave with wavelength %d is %f\n', wave_i, distance)
    
    % Plot x vs y for sinusoidal
    plot(x,y, LineWidth=3)
    
    % Fix the axis limits
    axis([x_min x_max y_min y_max])
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

    % Calculate the disance of the path
    distance = 0;
    for i = 2:length(x)
        distance = distance + sqrt((x(i)-x(i-1))^2 + (y(i)-y(i-1))^2);
    end
    all_distances(end+1) = distance;

    fprintf('Distance for square wave with wavelength %d is %f\n', wave_i, distance)
    
    % Plot values for the square wave
    plot(x, y, LineWidth=3)

end
    
% Show the legend
legend('Sinusoidal \lambda = 200mm', 'Square Wave \lambda = 200mm', 'Sinusoidal \lambda = 400mm', 'Square Wave \lambda = 400mm')
xlabel('x (mm)')
ylabel('y (mm)')

% print the distances
all_distances

% title('Search Paths')

%% Get x and y values for random search
% x = (x_max-x_min)*rand(resolution, 1)+x_min;
% y = (y_max-y_min)*rand(resolution, 1)+y_min;

% x(1) = 0;
% y(1) = 0;

% % Plot values for the random search
% figure
% plot(x, y)