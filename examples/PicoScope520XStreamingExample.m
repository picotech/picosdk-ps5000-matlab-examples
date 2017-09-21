%% PicoScope 5203 and 5204 Oscilloscope Streaming Data Capture Example
% This is a MATLAB script that demonstrates how to use the ps5000 API
% library functions to capture data in streaming mode from a PicoScope 5203
% or 5204 oscilloscope using the following approach:
%
% * Open a unit 
% * Display unit information 
% * Set up an input channel
% * Verify the timebase index
% * Setup a trigger
% * Output a signal from the signal generator
% * Set data buffers for collection
% * Start data collection
% * Retrieve the data values and convert to millivolts
% * Plot data live if the User chooses to do so
% * Plot data at the end of the capture
% * Close the unit
%
% *To run this example:*
% 	
% Type |PicoScope520XStreamingExample| at the MATLAB command prompt or run
% from the MATLAB Editor.
%
% *Copyright* © 2017 Pico Technology Ltd. See LICENSE file for terms.

%% Clear command window and close all figures

clc;
close all;

%% Load configuration information

PS5000Config;

%% Parameters used throughout the script

channelA    = ps5000Enuminfo.enPS5000Channel.PS5000_CHANNEL_A;
channelB    = ps5000Enuminfo.enPS5000Channel.PS5000_CHANNEL_B;

maxADCValue = 32512; % Maximum ADC count value
oversample  = 1;

%% Load library files
% Load the ps5000 and ps5000Wrap shared library files using the respective
% prototype files.

archStr = computer('arch');

ps5000MFile = str2func(strcat('ps5000MFile_', archStr));

if ~libisloaded('ps5000')

    if ispc()
        
        loadlibrary('ps5000.dll', ps5000MFile);
        
    elseif ismac()
        
        error('PS5000StreamingExample:OSNotSupported', ...
            'Mac OS X not supported, please contact Pico Technology Technical Support for further assistance.');
        
    elseif isunix()
        
        loadlibrary('libps5000.so', ps5000MFile, 'alias', 'ps5000');
        
    end

    if ~libisloaded('ps5000')
        
        error('PS5000StreamingExample:LibraryNotLoaded', 'Library ps5000 or ps5000MFile not found');
        
    end

end

ps5000WrapMFile = str2func(strcat('ps5000WrapMFile_', archStr));

if ~libisloaded('ps5000Wrap')

    if ispc()
        
        loadlibrary('ps5000Wrap.dll', ps5000WrapMFile);
        
    elseif ismac()
        
        error('PS5000StreamingExample:OSNotSupported', ...
            'Mac OS X not supported, please contact Pico Technology Technical Support for further assistance.');
        
    elseif isunix()
        
        loadlibrary('libps5000Wrap.so', ps5000WrapMFile, 'alias', 'ps5000Wrap');
        
    end

    if ~libisloaded('ps5000Wrap')
        
        error('PS5000StreamingExample:LibraryNotLoaded', 'Library ps5000Wrap or ps5000WrapMFile not found');
        
    end

end

% (Optional view library functions)
% libfunctionsview('ps5000Wrap');

%% Open unit
% Open connection to oscilloscope and obtain unique handle value for device. 

disp('PicoScope 5203 and 5204 Streaming Example');

unitHandle = 0;

[status.open, unitHandle] = calllib('ps5000', 'ps5000OpenUnit', unitHandle);

if (status.open > PicoStatus.PICO_OK)
   
    error('PS5000StreamingExample:DeviceNotOpened', 'Error opening device - status code %d \n', status.open);
    
end

%% Diplay unit information
% Display driver, variant and batch/serial number information

infoLine = blanks(100);
reqSize = length(infoLine);

% Obtain driver version
[status.infoDriver, driver]  = calllib('ps5000', ...
            'ps5000GetUnitInfo', unitHandle, infoLine, ... 
            length(infoLine), reqSize, PicoStatus.PICO_DRIVER_VERSION);
        
% Obtain variant information        
[status.infoVariant, variant]  = calllib('ps5000', ...
            'ps5000GetUnitInfo', unitHandle, infoLine, ... 
            length(infoLine), reqSize, PicoStatus.PICO_VARIANT_INFO);
        
% Obtain batch/serial information        
[status.infoVariant, serial]  = calllib('ps5000', ...
            'ps5000GetUnitInfo', unitHandle, infoLine, ... 
            length(infoLine), reqSize, PicoStatus.PICO_BATCH_AND_SERIAL);

fprintf('\nUnit information:\n\n');
fprintf('Driver : %s\n', driver);
fprintf('Variant: %s\n', variant);
fprintf('Serial : %s\n\n', serial);

% Obtain number of channels
channelCount = str2double(variant(2));

%% Set channels
% Set channel A to use DC coupling with an input range of ±2 volts 

channelSettings(1).enabled = PicoConstants.TRUE;
channelSettings(1).dc      = PicoConstants.TRUE;
channelSettings(1).range   = ps5000Enuminfo.enPS5000Range.PS5000_2V;

channelARangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(1).range + 1); % Used later in the script

[status.setChannelA]  = calllib('ps5000', 'ps5000SetChannel', unitHandle, channelA, channelSettings(1).enabled, ...
    channelSettings(1).dc, channelSettings(1).range);
        
if (status.setChannelA ~= PicoStatus.PICO_OK)
    
    error('PS5000StreamingExample:ChannelNotSet','ps5000SetChannel (A) - status code %d \n', status.setChannelA);
    
end

% Turn off channel B
channelSettings(2).enabled = PicoConstants.FALSE;
channelSettings(2).dc      = PicoConstants.TRUE;
channelSettings(2).range   = ps5000Enuminfo.enPS5000Range.PS5000_2V;

channelBRangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(2).range + 1); % Used later in the script

[status.setChannelB]  = calllib('ps5000', 'ps5000SetChannel', unitHandle, channelB, channelSettings(2).enabled, ...
    channelSettings(2).dc, channelSettings(2).range);
        
if(status.setChannelB ~= PicoStatus.PICO_OK)
    
    error('PS5000StreamingExample:ChannelNotSet', 'ps5000SetChannel (B) - status code %d \n', status.setChannelB);
    
end

% Set the number of enabled channels with the wrapper library
enabledChannels = zeros(PicoConstants.DUAL_SCOPE, 1, 'int16');

enabledChannels(1) = channelSettings(1).enabled;
enabledChannels(2) = channelSettings(2).enabled;

status.setEnabledChannels = calllib('ps5000Wrap', 'setEnabledChannels', unitHandle, enabledChannels);

%% Set up simple trigger
% Set a simple trigger on channel A - trigger when the signal rises through
% 500 mV, with an auto timeout of 5 seconds.

triggerEnabled  = PicoConstants.TRUE;
threshold       = mv2adc(500, channelARangeMv, maxADCValue);
direction       = ps5000Enuminfo.enThresholdDirection.RISING;
delay           = 0;
autoTriggerMs   = 5000; % 5 second auto trigger

[status.setSimpleTrigger] = calllib('ps5000', 'ps5000SetSimpleTrigger', unitHandle, triggerEnabled, ...
                                channelA, threshold, direction, delay, autoTriggerMs);
                            
%% Prompt to connect signal out to channel A

h = helpdlg('Connect Signal Out to channel A and click OK.', 'Connect Input Signal');
uiwait(h);
           
%% Start signal generator
% Output a sine wave at 1 Hz with a peak-to-peak voltage of 4 volts.

offsetVoltage   = 0; % Offset in microvolts
pkToPk          = 4000000; % Peak-to-peak amplitude in microvolts
waveType        = ps5000Enuminfo.enWaveType.PS5000_SINE; % Type of Wave. 
startFrequency  = 1; % Hz 
stopFrequency   = 1; % Hz Stop frequency must equal start frequency for constant waveform
increment       = 0; % Increment in frequency for sweep mode.
dwellTime       = 0; % Time (sec) spent in each frequency for sweep mode.
sweepType       = ps5000Enuminfo.enSweepType.UP; % Type of sweep. 
whiteNoise      = PicoConstants.FALSE;
shots           = 0;  
sweeps          = 0; 
triggerType     = ps5000Enuminfo.enSigGenTrigType.SIGGEN_RISING;
triggerSource   = ps5000Enuminfo.enSigGenTrigSource.SIGGEN_NONE;
extInThreshold  = 0; 

disp('Starting signal generator...');

status.setSigGenBuiltIn = calllib('ps5000', 'ps5000SetSigGenBuiltIn', unitHandle, ...
                            offsetVoltage, pkToPk, waveType, startFrequency, stopFrequency, increment, dwellTime,...
                            sweepType, whiteNoise, shots, sweeps, triggerType, triggerSource, extInThreshold);

%% Set data buffers
% Data buffers for channel A - buffers should be set with the driver,
% and these MUST be passed with application buffers to the wrapper library
% in order to ensure data is correctly copied.

sampleCount =  100000; % Size of the buffer to collect data from buffer.

ratioMode = ps5000Enuminfo.enRatioMode.RATIO_MODE_NONE;

% Buffers to be passed to the driver
pDriverBufferChA = libpointer('int16Ptr', zeros(sampleCount, 1, 'int16'));

status.setDataBufferChA = calllib('ps5000', 'ps5000SetDataBuffer', unitHandle, channelA, pDriverBufferChA, sampleCount);

% Application Buffers - these are for copying from the driver into.
pAppBufferChA = libpointer('int16Ptr', zeros(sampleCount, 1, 'int16'));

status.setAppDriverBuffersA = calllib('ps5000Wrap', 'setAppAndDriverBuffers', unitHandle, channelA, ...
                                pAppBufferChA, pDriverBufferChA, sampleCount);

%% Start streaming and collect data
% Collect data for 5 seconds after a trigger event, including any
% pre-trigger data - maximum array size will depend on the PC's resources -
% type <matlab:doc('memory') |memory|> at the MATLAB command prompt for
% further information.

% Define the sampling interval - this will be modified by the driver if the
% requested sampling interval is not supported

sampleInterval          = 1;
sampleIntervalTimeUnits = ps5000Enuminfo.enPS5000TimeUnits.PS5000_US;

% Set the number of pre- and post-trigger samples to collect. Note that in
% streaming mode, the device will begin returning data regardless of
% whether a trigger is set or not.
numPreTriggerSamples    = 0;
numPostTriggerSamples   = 5000000;

% Set other streaming parameters
autoStop            = PicoConstants.TRUE;
downSampleRatio     = 1;
overviewBufferSize  = sampleCount;

% Define buffers to store data collected from the channels. If capturing
% data without using the autoStop flag, or if using a trigger with the
% autoStop flag, allocate sufficient space (1.5 times the sum of the number of
% pre-trigger and post-trigger samples is shown below) to allow for
% additional pre-trigger data. Pre-allocating the array is more efficient
% than using <matlab:doc('vertcat') |vertcat|> to combine data.

maxSamples = numPreTriggerSamples + numPostTriggerSamples;

% Take into account the downSampleRatio
finalBufferLength = round(1.5 * maxSamples / downSampleRatio);

% Prompt User to indicate if they wish to plot live streaming data.
plotLiveData = questionDialog('Plot live streaming data?', 'Streaming Data Plot');

if (plotLiveData == PicoConstants.TRUE)
   
    disp('Live streaming data collection with second plot on completion.');
    
else
    
    disp('Streaming data plot on completion.');
    
end

[status.runStreaming, actualSampleInterval] = calllib('ps5000', 'ps5000RunStreaming', unitHandle, ...
                                                sampleInterval, sampleIntervalTimeUnits, numPreTriggerSamples, numPostTriggerSamples, ...
                                                autoStop, downSampleRatio, overviewBufferSize);
    
disp('Streaming data...');
fprintf('Click the STOP button to stop capture or wait for auto stop if enabled.\n\n') 

% Variables to be used when collecting the data:

hasAutoStopOccurred = PicoConstants.FALSE;  % Indicates if the device has stopped automatically.
newSamples          = 0;                    % Number of new samples returned from the driver.
previousTotal       = 0;                    % The previous total number of samples.
totalSamples        = 0;                    % Total samples captured by the device.
startIndex          = 0;                    % Start index of data in the buffer returned.
hasTriggered        = 0;                    % To indicate if trigger has occurred.
triggeredAtIndex    = 0;                    % The index in the overall buffer where the trigger occurred.

% Libpointer objects for streaming data parameters

pOverflow   = libpointer('int16Ptr', 0); 
pTrigAt     = libpointer('uint32Ptr', 0);
pAutoStop   = libpointer('int16Ptr', 0); 
pStartIndex = libpointer('uint32Ptr', 0); 

status.getStreamingLatestValues = PicoStatus.PICO_OK; % OK

% Display a 'Stop' button.
[stopFig.h, stopFig.h] = stopButton();             
             
flag = 1; % Use flag variable to indicate if stop button has been clicked (0)
setappdata(gcf, 'run', flag);

% Plot Properties - these are for displaying data as it is collected.

% Set x-axis for both live and post-capture plots according to the time
% units specified..
    
switch (sampleIntervalTimeUnits)

    case ps5000Enuminfo.enPS5000TimeUnits.PS5000_FS

        xLabelStr  = 'Time (fs)';

    case ps5000Enuminfo.enPS5000TimeUnits.PS5000_PS

        xLabelStr = 'Time (ps)';

    case ps5000Enuminfo.enPS5000TimeUnits.PS5000_NS

        xLabelStr = 'Time (ns)';

    case ps5000Enuminfo.enPS5000TimeUnits.PS5000_US

        xLabelStr = 'Time (\mus)';

    case ps5000Enuminfo.enPS5000TimeUnits.PS5000_MS

        xLabelStr = 'Time (ms)';

    case ps5000Enuminfo.enPS5000TimeUnits.PS5000_S

        xLabelStr = 'Time (s)';

    otherwise

        xLabelStr = 'Time';

end
    
if (plotLiveData == PicoConstants.TRUE)
    
    % Plot on a single figure 
    figure1 = figure('Name','PicoScope 5203 and 5204 Example - Streaming Mode Capture', ...
         'NumberTitle','off');

     axes1 = axes('Parent', figure1);

    % Estimate x-axis limit to try and avoid using too much CPU resources
    % when drawing - use max voltage range selected if plotting multiple
    % channels on the same graph.
    
    xlim(axes1, [0 (actualSampleInterval * finalBufferLength)]);

    yRange = channelARangeMv + 500;
    ylim(axes1,[(-1 * yRange) yRange]);

    hold(axes1,'on');
    grid(axes1, 'on');

    title(axes1, 'Live Streaming Data Capture');
    
    xlabel(axes1, xLabelStr);
    ylabel(axes1, 'Voltage (mV)');
    
end

% Collect samples as long as the |hasAutoStopOccurred| flag has not been
% set or the call to |getStreamingLatestValues()| does not return an error
% code (check for STOP button push inside loop).
while (hasAutoStopOccurred == PicoConstants.FALSE && status.getStreamingLatestValues == PicoStatus.PICO_OK)
    
    ready = PicoConstants.FALSE;
   
    while (ready == PicoConstants.FALSE)

        status.getStreamingLatestValues = calllib('ps5000Wrap', 'GetStreamingLatestValues', unitHandle);

        ready = calllib('ps5000Wrap', 'IsReady', unitHandle);

       % Give option to abort from here.
       flag = getappdata(gcf, 'run');
       drawnow;

       if (flag == 0)

           disp('STOP button clicked - aborting data collection.')
           break;

       end

       drawnow;

    end
	
    % Check for new data values.
    newSamples =  calllib('ps5000Wrap', 'AvailableData', unitHandle, pStartIndex);

    if (newSamples > 0)
        
        % Check if the scope has triggered.
        triggered = calllib('ps5000Wrap', 'IsTriggerReady', unitHandle, pTrigAt);

        if (triggered == PicoConstants.TRUE)

            hasTriggered    = PicoConstants.TRUE;
            triggeredAt     = pTrigAt.Value;
            
            % Adjust trigger position as MATLAB does not use zero-based
            % indexing.
            bufferTriggerPosition = triggeredAt + 1;
            
            fprintf('Triggered - index in buffer: %d\n', bufferTriggerPosition);

            hasTriggered = triggered;

            % Set the total number of samples at which the device
            % triggered.
            triggeredAtIndex = totalSamples + bufferTriggerPosition;

        end
        
        % Position index of data in the buffer(s).
        startIndex      = pStartIndex.Value;
        
        previousTotal   = totalSamples;
        totalSamples    = totalSamples + newSamples;

        % Printing to console can slow down acquisition - use for
        % demonstration.
        fprintf('Collected %d samples, start index: %d, total: %d.\n', newSamples, startIndex, totalSamples);
        
        firstValuePosn  = startIndex + 1;
        lastValuePosn   = startIndex + newSamples;
        
        % Convert data values to millivolts from the application buffer(s).
        bufferChAmV = adc2mv(pAppBufferChA.Value(firstValuePosn:lastValuePosn), channelARangeMv, maxADCValue);

        % Process collected data further if required - this example plots
        % the data if the User has selected 'Yes' at the prompt.
        
        % Copy data into the final buffer(s).
        pBufferChAFinal.Value((previousTotal + 1):totalSamples) = bufferChAmV;
        
        if (plotLiveData == PicoConstants.TRUE)
            
            % Time axis
            % Multiply by ratio mode as samples get reduced.
            time = (double(actualSampleInterval) * double(downSampleRatio)) * (previousTotal:(totalSamples - 1));
        
            plot(time, bufferChAmV);

        end
        
        % Clear variables for use again.
        clear bufferChAmV;
        clear firstValuePosn;
        clear lastValuePosn;
        clear startIndex;
        clear triggered;
        clear triggerAt;
   
    end
    
    % Check if auto stop has occurred.
    hasAutoStopOccurred = calllib('ps5000Wrap', 'AutoStopped', unitHandle);

    if (hasAutoStopOccurred == PicoConstants.TRUE)

       disp('AutoStop: TRUE - exiting loop.');
       break;

    end
   
    % Check if 'STOP' button pressed
    flag = getappdata(gcf, 'run');
    drawnow;

    if (flag == 0)

        disp('STOP button clicked - aborting data collection.')
        break;
        
    end
 
end

% Close the STOP button window.
if (exist('stopFig', 'var'))
    
    close('Stop Button');
    clear stopFig;
        
end

if (plotLiveData == PicoConstants.TRUE)
    
    drawnow;
    
    % Take hold off the current figure.
    hold(axes1, 'off');
    movegui(figure1, 'west');
    
end

if (hasTriggered == PicoConstants.TRUE)
   
    fprintf('Triggered at overall index: %d\n', triggeredAtIndex);
    
end

fprintf('\n');

%% Stop the device

[status.stop] = calllib('ps5000', 'ps5000Stop', unitHandle);

%% Find the number of samples.
% This is the number of samples held in the driver itself. The actual
% number of samples collected when using a trigger is likely to be greater.
% In this example, the total number of samples collected will be used.

pNumStreamingValues         = libpointer('uint32Ptr', 0);

status.numStreamingValues   = calllib('ps5000', 'ps5000NoOfStreamingValues', unitHandle, pNumStreamingValues);

numStreamingValues          = pNumStreamingValues.Value;

fprintf('Number of samples available from the driver: %u.\n\n', numStreamingValues);

%% Process data
% Process data post-capture if required - here the data will be plotted.

% Reduce size of arrays if required.
if (totalSamples < maxSamples)
    
    pBufferChAFinal.Value(totalSamples + 1:end) = [];
 
end

% Retrieve data for the channels.
channelAFinal = pBufferChAFinal.Value;

% Plot total data collected on another figure.
finalFigure = figure('Name','PicoScope 5203 and 5204 Example - Streaming Mode Capture', ...
    'NumberTitle','off');

finalFigureAxes = axes('Parent', finalFigure);
movegui(finalFigure, 'east');
hold on;

title(finalFigureAxes, 'Streaming Data Acquisition (Final)');
xlabel(finalFigureAxes, xLabelStr);
ylabel(finalFigureAxes, 'Voltage (mV)');

maxYRange = channelARangeMv + 500;
ylim(finalFigureAxes,[(-1 * maxYRange) maxYRange]);

% Calculate values for time axis, then plot.
timeAxis = (double(actualSampleInterval) * double(downSampleRatio)) * (0:totalSamples - 1);
plot(finalFigureAxes, timeAxis(1:totalSamples), channelAFinal(1:totalSamples));

if (hasTriggered)
   
    plot(finalFigureAxes, timeAxis(triggeredAtIndex), channelAFinal(triggeredAtIndex), 'rx', 'MarkerSize', 10);
    
end

grid(finalFigureAxes, 'on');
legend(finalFigureAxes, 'Channel A');
hold(finalFigureAxes, 'off');

%% Close unit

[status.closeUnit] = calllib('ps5000','ps5000CloseUnit', unitHandle);

if (status.closeUnit == PicoStatus.PICO_OK)
   
    disp('Unit closed successfully.')
    
else
    
    error('PS5000StreamingExample:CloseUnitError', 'ps5000CloseUnit - status code %d \n', status.closeUnit);
    
end

%% Unload library files

unloadlibrary('ps5000');

if (~libisloaded('ps5000'))
   
    disp('ps5000 library unloaded successfully.');
    
else
    
    error('PS5000StreamingExample:LibraryUnloadError', 'ps5000 library not unloaded.');
    
end

unloadlibrary('ps5000Wrap');

if (~libisloaded('ps5000Wrap'))
   
    disp('ps5000Wrap library unloaded successfully.');
    
else
    
    error('PS5000StreamingExample:LibraryUnloadError', 'ps5000Wrap library not unloaded.');
    
end