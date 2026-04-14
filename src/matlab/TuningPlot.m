% Matlab Script to Read and Plot Serial Data
%
%           Version 4.0 (11/01/2018) H.C.
%           Version 5.0 (09/24/2020) H.C.
%           Version 6.0 (10/13/2021) H.C.
%           Version 7.0 (02/21/2022) H.C.
%

% make sure no residual serial object in Matlab workspace
if (exist('s1','var'))
    delete(s1);
    clear s1;
end

% clear all figures and variables in workspace
close all
clear data data_char status
delete(instrfind);

% change data labels for the 3 signals here
labels = {'Setpoint','Position'};

try

% define serial object with matching com port and baud rate
% TODO: Change COM port to match VSCode
s1 = serialport("COM18",115200);       % Windows
% s1 = serialport("/dev/cu.usbmodem144101",115200);        % MacOS

disp(' ');
disp('*** Serial Data Capture ***');
disp('*** Press the STOP button to end ***');
disp(' ');

status = getpinstatus(s1);
%configureTerminator(s1,"LF");

% initialize variables
i = 1;

% create a STOP button on the figure window
hFig = figure(1);
hFig.Name = 'Serial Data Plot';
ax = axes(hFig);
ax.Units = 'normalized';
ax.Position = [0.125 0.15 0.775 0.775];

hButton = uicontrol(hFig,'Style','pushbutton');
hButton.String = 'STOP';
hButton.BackgroundColor = [1 0 0];
hButton.ForegroundColor = [1 1 1];
hButton.FontWeight = 'bold';
hButton.UserData = 0;

% here we define 3 data lines, add or substract lines if needed
h1 = animatedline ('Color','g');
h2 = animatedline ('Color','b');

title('Streaming Serial Data <Press STOP button to end>')
xlabel('Time (sec)'),ylabel('Values'), grid
legend(labels, 'Location', 'northwest');

% get data from the serial object till the STOP button is pressed
while ( hButton.UserData == 0 )
    
    data_char{i} = readline(s1);
    
    if(~strcmp(data_char{i},''))
        data(i,:) = str2num(data_char{i});
    else
        break;
    end
    
    addpoints(h1, data(i,1), data(i,2));
    addpoints(h2, data(i,1), data(i,3));
    legend(labels, 'Location', 'northwest');

    drawnow limitrate
    hButton.Callback = ['hButton.UserData = 1;','disp(''Stopping data collection...'')'];
    i = i+1;
end

% drawnow;
fprintf(['*** Done. A total of ',num2str(i-1), ' datasets collected ***\n']);
disp(' ');

% re-plot the data once the data collection is done to get all figure features
figure(1)
plot(data(:,1), data(:,2),'g', data(:,1), data(:,3),'b')
title('Serial Data')
xlabel('Time (sec)'),ylabel('Values'), grid
legend(labels, 'Location', 'northwest');

dt = mean(diff(data(:,1)));
disp(['Sampling period (sec) = ', num2str(dt)])
disp(['Sampling frequency (Hz) = ', num2str(1/dt)])

% To disconnect the serial port object from the serial port
delete(s1);
clear s1;

catch ME
    warn_string = ["1. Check if Serial Port ID is set correctly.",...
        "2. Make sure Serial Monitor or Plotter is not running.",...
        "3. #define MatlabPlot in PlatformIO code is enabled.",... 
        "4. Do not close the figure window while the program is running.",...
        "5. Hit a key on the keyboard while the real-time graph",...
        "     is active to exit the program properly."];   
    warndlg(warn_string, 'Serial Read Warning');
    disp(' ');
    disp('Program terminated abnormally!');
    disp(['Cause: ' ME.identifier]);
    if (exist('s1','var'))
        delete(s1);
        clear s1;
    end
end

% Use the command below to force clear any hidden com port if needed
% delete(instrfind);
