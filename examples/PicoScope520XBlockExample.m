%% PicoScope 5203 and 5204 Oscilloscope Block Data Capture Example
% This is a MATLAB script that demonstrates how to use the ps5000 API
% library functions to capture a block of data from a PicoScope 5203 or
% 5204 oscilloscope using the following approach:
%
% * Open a unit 
% * Display unit information 
% * Set up an input channel
% * Verify the timebase index
% * Setup a trigger
% * Output a signal from the signal generator
% * Collect a block of data
% * Retrieve the data values and convert to millivolts
% * Plot data
% * Close the unit
%
% *To run this example:*
% 	
% Type |PicoScope520XBlockExample| at the MATLAB command prompt or run from the
% MATLAB Editor.
%
% *Copyright* © 2015-2017 Pico Technology Ltd. See LICENSE file for terms.

%% Clear command window and close all figures

clc;
close all;

%% Load configuration information

PS5000Config;

%% Parameters used throughout the script

channelA    = ps5000Enuminfo.enPS5000Channel.PS5000_CHANNEL_A;

maxADCValue = 32512; % Maximum ADC count value
oversample  = 1;

%% Load library file
% Load the ps5000 shared library using the prototype file.

archStr = computer('arch');

ps5000MFile = str2func(strcat('ps5000MFile_', archStr));

if ~libisloaded('ps5000')

    if ispc()
        
        loadlibrary('ps5000.dll', ps5000MFile);
        
    elseif ismac()
        
        error('PS5000BlockExample:OSNotSupported', ...
            'Mac OS X not supported, please contact Pico Technology Technical Support for further assistance.');
        
    elseif isunix()
        
        loadlibrary('libps5000.so', ps5000MFile, 'alias', 'ps5000');
        
    end

    if ~libisloaded('ps5000')
        
        error('PS5000BlockExample:LibraryNotLoaded', 'Library ps5000 or ps5000MFile not found');
        
    end

end

% (Optional view library functions)
% libfunctionsview('ps5000');

%% Open unit
% Open connection to oscilloscope and obtain unique handle value for device. 

disp('PicoScope 5203 and 5204 Block Example');

unitHandle = 0;

[status.open, unitHandle] = calllib('ps5000', 'ps5000OpenUnit', unitHandle);

if (status.open > PicoStatus.PICO_OK)
   
    error('PS5000BlockExample:DeviceNotOpened', 'Error opening device - status code %d \n', status.open);
    
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

%% Set channels
% Set channel A to use DC coupling with an input range of ±2 V 

enabled         = PicoConstants.TRUE;
dc              = PicoConstants.TRUE;
channelARange   = ps5000Enuminfo.enPS5000Range.PS5000_2V;
channelARangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelARange + 1); % Used later in the script

[status.setChannelA]  = calllib('ps5000', 'ps5000SetChannel', unitHandle, channelA, enabled, dc, channelARange);
        
if (status.setChannelA ~= PicoStatus.PICO_OK)
    
    error('PS5000BlockExample:ChannelNotSet', 'ps5000SetChannel - status code %d \n', status.setChannelA);
    
end

%% Verify timebase and maximum number of samples
% Use the |ps5000GetTimebase()| function to query the driver as to the
% suitability of using a particular timebase index and the maximum number
% of samples available in the segment selected.
%
% To use the fastest sampling interval possible, enable one analog
% channel and turn off the other channel.
%
% Use a while loop to query the function until the status indicates that a
% valid timebase index has been selected. In this example, the timebase
% index of 127 is valid.
%
% For further information on timebase indices, please refer to the
% Timebases section in the the
% <https://www.picotech.com/download/manuals/ps5000pg-en-1.pdf PicoScope
% 5000 Series PC Oscilloscopes Programmer's Guide>.

preTriggerSamples   = 0;
postTriggerSamples  = 1000000;

timebaseIndex           = 127; % 1us
numSamples              = preTriggerSamples + postTriggerSamples;
timeIntervalNanoseconds = 0;
maxSamples              = 0;
segmentIndex            = 0;

status.getTimebase = PicoStatus.PICO_INVALID_TIMEBASE;

% If timebase is not valid, increment until a suitable timebase has been
% found.
while (status.getTimebase == PicoStatus.PICO_INVALID_TIMEBASE)
    
    [status.getTimebase, timeIntervalNanoseconds, maxSamples]  = calllib('ps5000','ps5000GetTimebase', unitHandle, timebaseIndex, ...
                                                                    numSamples, timeIntervalNanoseconds, oversample, maxSamples, segmentIndex);
    
    if (status.getTimebase == PicoStatus.PICO_OK)
       
        break;
        
    else
        
        timebaseIndex = timebaseIndex + 1;
        
    end

end

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
% Output a sine wave at 5 Hz with a peak-to-peak voltage of 3 Volts.

offsetVoltage   = 0; % Offset in microvolts
pkToPk          = 3000000; % Peak-to-peak amplitude in microvolts
waveType        = ps5000Enuminfo.enWaveType.PS5000_SINE; % Type of Wave. 
startFrequency  = 5; % Hz 
stopFrequency   = 5; % Hz Stop frequency must equal start frequency for constant waveform
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

%% Collect block of data
% Start data collection and poll the driver until the device is ready.

disp('Collecting data...');

timeIndisposedMs = 0;
lpReady = [];
pParameter = [];

[status.runBlock, timeIndisposedMs] = calllib('ps5000','ps5000RunBlock', unitHandle, ...
                                        preTriggerSamples, postTriggerSamples, timebaseIndex, oversample, timeIndisposedMs, ...
                                        segmentIndex, lpReady, pParameter);    

% Poll the driver to check if the device has completed data collection.

ready = 0;

while (ready == 0)

    [status.isReady, ready] = calllib('ps5000', 'ps5000IsReady', unitHandle, ready);
   
    pause(0.01);
    
end

%% Retrieve data values
% Set up data buffer(s) for the channel and retrieve data from the driver.
% For better efficiency when collecting multiple blocks of data, set up the
% data buffers before starting the capture.

pBufferA = libpointer('int16Ptr', zeros(numSamples, 1, 'int16'));

[status.setDataBufferA] = calllib('ps5000', 'ps5000SetDataBuffer', unitHandle, ...
                            channelA, pBufferA, numSamples);       
 
% Get values

startIndex          = 0;
downSampleRatio     = 1;
downSampleRatioMode = ps5000Enuminfo.enRatioMode.RATIO_MODE_NONE;
pOverflow           = libpointer('int16Ptr', 0);

[status.getValues, numSamplesCollected] = calllib('ps5000', 'ps5000GetValues', unitHandle, ...
                                                        startIndex, numSamples, downSampleRatio, downSampleRatioMode, segmentIndex, pOverflow);

if (status.getValues ~= PicoStatus.PICO_OK)
   
    error('PS5000BlockExample:GetValuesError', 'ps5000GetValues - status code %d \n', status.getValues);
    
else
    
    disp('Data values retrieved.');
    overflow = pOverflow.Value;
    
end

%% Process data
% In this example, the data collected from the device will be converted to
% millvolts and displayed on a plot.

% Convert data to millivolts
% Use the |adc2mv| function from the
% <https://uk.mathworks.com/matlabcentral/fileexchange/53681-picoscope-support-toolbox
% PicoScope Support Toolbox>.

chA = adc2mv(pBufferA.Value, channelARangeMv, maxADCValue);

% Calculate time axis values (in nanoseconds) and convert to milliseconds. 
% Use |timeIntervalNanoSeconds| output from |ps5000GetTimebase()| or calculate
% using the <https://www.picotech.com/download/manuals/ps5000pg-en-1.pdf PicoScope 5000 Series PC Oscilloscopes Programmer's Guide>.

disp('Plotting data...');

timeNs = double(timeIntervalNanoseconds) * double(0:numSamplesCollected - 1);
timeMs = timeNs / 1e6;

figure1 = figure('Name','PicoScope 5203 and 5204 Example - Block Mode Capture', ...
    'NumberTitle', 'off');

% Channel A
plot(timeMs, chA, 'b');
ylim([(-1 * channelARangeMv) channelARangeMv]);
title('Channel A', 'FontWeight', 'bold');
xlabel('Time (ms)');
ylabel('Voltage (mV)');
grid on;

%% Stop the device

[status.stop] = calllib('ps5000', 'ps5000Stop', unitHandle);

%% Close unit

[status.closeUnit] = calllib('ps5000','ps5000CloseUnit', unitHandle);

if (status.closeUnit == PicoStatus.PICO_OK)
   
    disp('Unit closed successfully.')
    
else
    
    error('PS5000BlockExample:CloseUnitError', 'ps5000CloseUnit - status code %d \n', status.closeUnit);
    
end

%% Unload library files

unloadlibrary('ps5000');

if (~libisloaded('ps5000'))
   
    disp('ps5000 library unloaded successfully.');
    
else
    
    error('PS5000BlockExample:LibraryUnloadError', 'Library not unloaded.');
    
end