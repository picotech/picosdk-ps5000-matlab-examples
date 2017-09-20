%% PicoScope 5203 and 5204 Oscilloscope Rapid Block Data Capture Example
% This is a MATLAB script that demonstrates how to use the ps5000 API
% library functions to capture data using rapid block mode from a
% PicoScope 5203 or 5204 oscilloscope using the following approach:
%
% * Open a unit 
% * Display unit information 
% * Set up an input channel
% * Segment the memory
% * Verify the timebase index
% * Setup a trigger
% * Setup data buffers for collection
% * Output a signal from the signal generator
% * Collect data in rapid block mode
% * Retrieve data values from the device and convert to millivolts
% * Plot data
% * Close the unit.
%
% *To run this example:*
%
% Type |PicoScope520XRapidBlockExample| at the MATLAB command prompt or run from the
% MATLAB Editor.
%
% *Copyright* © 2016-2017 Pico Technology Ltd. See LICENSE file for terms.

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

%% Load library file
% Load the ps5000 shared library using the appropriate prototype file.

archStr = computer('arch');

ps5000MFile = str2func(strcat('ps5000MFile_', archStr));
    
if ~libisloaded('ps5000')

    if ispc()
        
        loadlibrary('ps5000.dll', ps5000MFile);
        
    elseif ismac()
        
        error('PS5000RapidBlockExample:OSNotSupported', ...
            'Mac OS X not supported, please contact Pico Technology Technical Support for further assistance.');
        
    elseif isunix()
        
        loadlibrary('libps5000.so', ps5000MFile, 'alias', 'ps5000');
        
    end

    if ~libisloaded('ps5000')
        
        error('PS5000RapidBlockExample:LibraryNotLoaded', 'Library ps5000 or ps5000MFile not found');
        
    end

end

% (Optional view library functions)
% libfunctionsview('ps5000');

%% Open unit
% Open connection to oscilloscope and obtain unique handle value for device. 

disp('PicoScope 5203 and 5204 Rapid Block Example');

unitHandle = 0;

[status.open, unitHandle] = calllib('ps5000', 'ps5000OpenUnit', unitHandle);

if(status.open > PicoStatus.PICO_OK)
   
    error('PS5000RapidBlockExample:DeviceNotOpened', 'Error opening device - status code %d \n', status.open);
    
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

fprintf('\nUnit information:-\n\n');
fprintf('Driver : %s\n', driver);
fprintf('Variant: %s\n', variant);
fprintf('Serial : %s\n\n', serial);

% Obtain number of channels
channelCount = str2double(variant(2));

%% Set channels
% Set channel A to use DC coupling with an input range of +/-2 V 

channelSettings(1).enabled = PicoConstants.TRUE;
channelSettings(1).dc      = PicoConstants.TRUE;
channelSettings(1).range   = ps5000Enuminfo.enPS5000Range.PS5000_2V;

channelARangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(1).range + 1); % Used later in the script

[status.setChannelA]  = calllib('ps5000', 'ps5000SetChannel', unitHandle, channelA, channelSettings(1).enabled, ...
    channelSettings(1).dc, channelSettings(1).range);
        
if (status.setChannelA ~= PicoStatus.PICO_OK)
    
    error('PS5000RapidBlockExample:ChannelNotSet','ps5000SetChannel (A) - status code %d \n', status.setChannelA);
    
end

% Turn off channel B
channelSettings(2).enabled = PicoConstants.FALSE;
channelSettings(2).dc      = PicoConstants.TRUE;
channelSettings(2).range   = ps5000Enuminfo.enPS5000Range.PS5000_2V;

channelBRangeMv = PicoConstants.SCOPE_INPUT_RANGES(channelSettings(2).range + 1); % Used later in the script

[status.setChannelB]  = calllib('ps5000', 'ps5000SetChannel', unitHandle, channelB, channelSettings(2).enabled, ...
    channelSettings(2).dc, channelSettings(2).range);
        
if(status.setChannelB ~= PicoStatus.PICO_OK)
    
    error('PS5000RapidBlockExample:ChannelNotSet', 'ps5000SetChannel (B) - status code %d \n', status.setChannelB);
    
end

%% Segment the memory and set the number of captures
% Divide the buffer memory of the device into segments and indicate the
% number of captures to be obtained.

nSegments   = 32768;
pNMaxSamples = libpointer('int32Ptr', 0);

[status.memorySegments] = calllib('ps5000', 'ps5000MemorySegments', unitHandle, nSegments, pNMaxSamples);

nMaxSamples = pNMaxSamples.Value;

% Set number of captures - can be less than or equal to the number of segments.
nCaptures = 2000;

if (nCaptures > nSegments)
   
    nCaptures = nSegments;
    
end

[status.setnoofCaptures] = calllib('ps5000','ps5000SetNoOfCaptures', unitHandle, nCaptures);

%% Verify timebase and maximum number of samples
% Use the |ps5000aGetTimebase()| function to query the driver as to the
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
postTriggerSamples  = 1000;

timebaseIndex           = 127; % 1 us
numSamples              = preTriggerSamples + postTriggerSamples;
timeIntervalNanoseconds = 0;
maxSamples              = 0;
segmentIndex            = 0;

status.getTimebase = PicoStatus.PICO_INVALID_TIMEBASE;

% If timebase is not valid, increment until a suitable timebase has been
% found.
while (status.getTimebase == PicoStatus.PICO_INVALID_TIMEBASE)
    
    [status.getTimebase, timeIntervalNanoseconds, maxSamples]  = calllib('ps5000', 'ps5000GetTimebase', unitHandle, timebaseIndex, ...
        numSamples, timeIntervalNanoseconds, oversample, maxSamples, segmentIndex);
    
    if (status.getTimebase == PicoStatus.PICO_OK)
       
        break;
        
    else
        
        timebaseIndex = timebaseIndex + 1;
        
    end

end

% Number of samples to collect per channel, per segment must be less than the maximum
% number of samples available per channel for each segment.

% Find the number of channels that are enabled

if (channelSettings(1).enabled && channelSettings(2).enabled)
    
    numEnabledChannels = 2;
    
elseif (channelSettings(1).enabled || channelSettings(2).enabled)
   
    numEnabledChannels = 1;
    
end

nMaxSamplesPerChannel = nMaxSamples / numEnabledChannels;

if (preTriggerSamples + postTriggerSamples > nMaxSamplesPerChannel)
    
    warning('PS5000RapidBlock:MaxSamplesExceeded', ['Number of pre-trigger and post-trigger samples per segment ' ...
        'exceeds maximum number of samples available for each channel per segment.']);
    
    % Change the number of post-trigger samples to match.
    
    if (preTriggerSamples < nMaxSamplesPerChannel)
       
        postTriggerSamples = nMaxSamplesPerChannel - preTriggerSamples;
        
    elseif (preTriggerSamples == nMaxSamplesPerChannel)
        
        postTriggerSamples = 0;
        
    else
        
        preTriggerSamples   = floor(nMaxSamplesPerChannel / 2);
        postTriggerSamples  = floor(nMaxSamplesPerChannel / 2);
        
    end
    
    fprintf('Changed number of samples:-\n\nPre-trigger samples: %d, Post-trigger samples: %d\n\n', preTriggerSamples, postTriggerSamples);

end 

numSamples = preTriggerSamples + postTriggerSamples;

%% Set up simple trigger
% Set a simple trigger on channel A - trigger when the signal rises through
% 500 mV, with an auto timeout of 5 seconds.

triggerEnabled  = PicoConstants.TRUE;
threshold       = mv2adc(500, channelARangeMv, maxADCValue);
direction       = ps5000Enuminfo.enThresholdDirection.RISING;
delay           = 0;
autoTriggerMs   = 5000; % 5 second auto trigger

[status.setSimpleTrigger]  = calllib('ps5000', 'ps5000SetSimpleTrigger', unitHandle, triggerEnabled, ...
    channelA, threshold, direction, delay, autoTriggerMs);

%% Set up data buffers
% It can be more efficient to set up data buffers prior to data collection,
% particularly for multiple data sets.

% Set up a 2D array of libpointer objects corresponding to segment x
% channel

% Initialise variable for array of libpointers
pBuffer(nCaptures, channelCount) = libpointer;

% Initialise array for status values
status.setDataBufferBulk = zeros(nCaptures, channelCount, 'uint32');

for ch = 1:channelCount
    
    % Set Data Buffer for each segment if channel is enabled
    if (channelSettings(ch).enabled == PicoConstants.TRUE) 
   
        for segment = 1:nCaptures 
    
            pBuffer(segment, ch) = libpointer('int16Ptr', zeros(numSamples, 1, 'int16'));
            
            status.setDataBufferBulk(segment, ch) = calllib('ps5000', 'ps5000SetDataBufferBulk', unitHandle, ...
                        (ch - 1), pBuffer(segment, ch), numSamples, (segment - 1));
                    
            if (status.setDataBufferBulk(segment, ch) ~= PicoStatus.PICO_OK)

                error('PS5000RapidBlockExample:SetDataBufferBulkError', 'Set Data Buffer for Channel %d Segment % error code %d', ...
                            (ch - 1), (segment - 1), status.setDataBufferBulk(segment, ch));

            end       
        
        end
        
   end
    
end

%% Prompt to connect signal out to channel A

h = helpdlg('Connect Signal Out to channel A and click OK.', 'Connect Input Signal');
uiwait(h);

%% Start signal generator
% Output a swept sine wave starting at 2 kHz with a peak-to-peak voltage of 3 volts.
% Stop frequency is 5 kHz, with an increment of 250 Hz and dwell time of
% 0.01 seconds.

offsetVoltage   = 0; % Offset in microvolts
pkToPk          = 3000000; % Peak-to-peak amplitude in microvolts
waveType        = ps5000Enuminfo.enWaveType.PS5000_SINE; % Type of wave. 
startFrequency  = 2000; % Hz 
stopFrequency   = 5000; % Hz Stop frequency must equal start frequency for constant waveform
increment       = 250; % Increment in frequency for sweep mode.
dwellTime       = 0.01; % Time (sec) spent in each frequency for sweep mode.
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
                        
if (status.setSigGenBuiltIn ~= PicoStatus.PICO_OK)
   
    error('PS5000RapidBlockExample:SigGenBuiltInError', 'Error setting signal generator');
    
end

%% Collect rapid block data
% Start data collection and poll the driver until the device is ready.

disp('Collecting data...');

timeIndisposedMs = 0;
lpReady = [];
pParameter = [];

[status.runBlock, timeIndisposedMs]  = calllib('ps5000','ps5000RunBlock', unitHandle, ...
    preTriggerSamples, postTriggerSamples, timebaseIndex, oversample, timeIndisposedMs, ...
    segmentIndex, lpReady, pParameter);    

% Poll the driver to check if the device has completed data collection.

ready = 0;

while (ready == 0)

    [status.isReady, ready] = calllib('ps5000', 'ps5000IsReady', unitHandle, ready);
   
    pause(0.01);
    
end

%% Retrieve data values
% Retrieve the data values for the waveforms from the device.

fromSegmentIndex    = 0;
toSegmentIndex      = nCaptures - 1;
pOverflowBuffer     = libpointer('int16Ptr', zeros(nCaptures, 1, 'int16')); % Pointer to array

% Retrieve data values
[status.getValuesBulk, numSamplesCollected]  = calllib('ps5000', 'ps5000GetValuesBulk', unitHandle, ...
                                                    numSamples, fromSegmentIndex, toSegmentIndex, pOverflowBuffer);

if (status.getValuesBulk ~= PicoStatus.PICO_OK)
   
    error('PS5000RapidBlockExample:GetValuesBulkError', 'Error retrieving data values.');
    
else
    
    disp('Data values retrieved.');
    
end

%% Process Data
% In this example, the data collected from the device will be converted to
% millivolts and displayed on a plot.

% Convert data to millivolts.
% Use the |adc2mv| function from the
% <https://uk.mathworks.com/matlabcentral/fileexchange/53681-picoscope-support-toolbox PicoScope Support
% Toolbox>.

% Create empty arrays for channel data.
channelAData = [];
channelBData = [];

 % Retrieve data values for enabled channels
for ch = 1:channelCount

    if (channelSettings(ch).enabled == PicoConstants.TRUE)

        % Obtain a cell array representing the waveforms for the channel
                    
        bufferCell = get(pBuffer(:, ch), 'Value');
                    
        % Combine the cell contents into a nCaptures x numSamples array
                    
        bufferMatrix = [];
                    
        if (nCaptures > 1)

            bufferMatrix = horzcat(bufferCell{1:end})';

        else

            bufferMatrix = bufferCell;

        end
        
        switch (ch - 1)
            
            case channelA
                
                channelAData = adc2mv(bufferMatrix, channelARangeMv, maxADCValue);
                
            case channelB
            
                channelBData = adc2mv(bufferMatrix, channelBRangeMv, maxADCValue);
                
        end
       
    end
    
end

% Calculate time axis values (in nanoseconds) and convert to milliseconds.
% Use |timeIntervalNanoSeconds| output from |ps5000GetTimebase()| or
% calculate using the
% <https://www.picotech.com/download/manuals/ps5000pg-en-1.pdf PicoScope
% 5000 Series PC Oscilloscopes Programmer's Guide>.

disp('Plotting data...');

timeNs = double(timeIntervalNanoseconds) * double(0:numSamplesCollected - 1);

% Plot channel A data
figure1 = figure('Name','PicoScope 5203 and 5204 Example - Rapid Block Mode Capture', ...
    'NumberTitle', 'off');

axes1 = axes('Parent', figure1);
view(axes1,[-15 24]);
grid(axes1,'on');
hold(axes1,'all');

% Plot the first 15 waveforms

for i = 1:15
    
    plot3(timeNs, i * (ones(numSamplesCollected, 1)), channelAData(i, :));
    
end

zlim(axes1, [(-1 * channelARangeMv) channelARangeMv]);
title(axes1, 'Channel A', 'FontWeight', 'bold');
xlabel(axes1, 'Time (ns)');
ylabel(axes1, 'Capture');
zlabel(axes1, 'Voltage (mV)');
grid(axes1, 'on');

hold(axes1, 'off');

%% Stop the device

[status.stop] = calllib('ps5000', 'ps5000Stop', unitHandle);

%% Close unit

[status.closeUnit] = calllib('ps5000','ps5000CloseUnit', unitHandle);

if (status.closeUnit == PicoStatus.PICO_OK)
   
    disp('Unit closed successfully.')
    
else
    
    error('PS5000RapidBlockExample:CloseUnitError', 'ps5000CloseUnit - status code %d \n', status.closeUnit);
    
end

%% Unload library files

unloadlibrary('ps5000');

if (~libisloaded('ps5000'))
   
    disp('ps5000 library unloaded successfully');
    
else
    
    error('PS5000RapidBlockExample:LibraryUnloadError', 'Library not unloaded.');
    
end