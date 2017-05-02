%% PS5000Config Configure path information
% Configures paths according to platforms and loads information from
% prototype files for PicoScope 5203 abd 5204 Oscilloscopes. The folder 
% that this file is located in must be added to the MATLAB path.
%
% Platform Specific Information:-
%
% Microsoft Windows: Download the Software Development Kit installer from
% the <a href="matlab: web('https://www.picotech.com/downloads')">Pico Technology Download software and manuals for oscilloscopes and data loggers</a> page.
% 
% Linux: Follow the instructions to install the libps5000 and libpswrappers
% packages from the <a href="matlab:
% web('https://www.picotech.com/downloads/linux')">Pico Technology Linux Software & Drivers for Oscilloscopes and Data Loggers</a> page.
%
% Apple Mac OS X: Follow the instructions to install the PicoScope 6
% application from the <a href="matlab: web('https://www.picotech.com/downloads')">Pico Technology Download software and manuals for oscilloscopes and data loggers</a> page.
% Optionally, create a 'maci64' folder in the same directory as this file
% and copy the following files into it:
%
% * libps5000.dylib and any other libps5000 library files
% * libps5000Wrap.dylib and any other libps5000Wrap library files
% * libpicoipp.dylib and any other libpicoipp library files
% * libiomp5.dylib
%
% Contact our Technical Support team via the <a href="matlab: web('https://www.picotech.com/tech-support/')">Technical Enquiries form</a> for further assistance.
%
% Run this script in the MATLAB environment prior to connecting to the 
% device.
%
% This file can be edited to suit application requirements.

%% Set Path to Shared Libraries
% Set paths to shared library files according to the operating system and
% architecture.

% Identify working directory
ps5000ConfigInfo.workingDir = pwd;

% Find file name
ps5000ConfigInfo.configFileName = mfilename('fullpath');

% Only require the path to the config file
[ps5000ConfigInfo.pathStr] = fileparts(ps5000ConfigInfo.configFileName);

% Identify architecture e.g. 'win64'
ps5000ConfigInfo.archStr = computer('arch');

try

    addpath(fullfile(ps5000ConfigInfo.pathStr, ps5000ConfigInfo.archStr));
    
catch err
    
    error('PS5000Config:OperatingSystemNotSupported', 'Operating system not supported - please contact support@picotech.com');
    
end

% Set the path according to operating system.

if(ismac())
    
    % Libraries (including wrapper libraries) are stored in the PicoScope
    % 6 App folder. Add locations of library files to environment variable.
    
    setenv('DYLD_LIBRARY_PATH', '/Applications/PicoScope6.app/Contents/Resources/lib');
    
    if(strfind(getenv('DYLD_LIBRARY_PATH'), '/Applications/PicoScope6.app/Contents/Resources/lib'))
       
        addpath('/Applications/PicoScope6.app/Contents/Resources/lib');
        
    else
        
        warning('PS5000Config:LibraryPathNotFound','Locations of libraries not found in DYLD_LIBRARY_PATH');
        
    end
    
elseif(isunix())
	    
    % Edit to specify location of .so files or place .so files in same directory
    addpath('/opt/picoscope/lib/'); 
		
elseif(ispc())
    
    % Microsoft Windows operating systems
    
    % Set path to dll files if the Pico Technology SDK Installer has been
    % used or place dll files in the folder corresponding to the
    % architecture. Detect if 32-bit version of MATLAB on 64-bit Microsoft
    % Windows.
    
    ps5000ConfigInfo.winSDKInstallPath = '';
    
    if(strcmp(ps5000ConfigInfo.archStr, 'win32') && exist('C:\Program Files (x86)\', 'dir') == 7)
       
        try 
            
            addpath('C:\Program Files (x86)\Pico Technology\SDK\lib\');
            ps5000ConfigInfo.winSDKInstallPath = 'C:\Program Files (x86)\Pico Technology\SDK';
            
        catch err
           
            warning('PS5000Config:DirectoryNotFound', ['Folder C:\Program Files (x86)\Pico Technology\SDK\lib\ not found. '...
                'Please ensure that the location of the library files are on the MATLAB path.']);
            
        end
        
    else
        
        % 32-bit MATLAB on 32-bit Windows or 64-bit MATLAB on 64-bit
        % Windows operating systems
        try 
        
            addpath('C:\Program Files\Pico Technology\SDK\lib\');
            ps5000ConfigInfo.winSDKInstallPath = 'C:\Program Files\Pico Technology\SDK';
            
        catch err
           
            warning('PS5000Config:DirectoryNotFound', ['Folder C:\Program Files\Pico Technology\SDK\lib\ not found. '...
                'Please ensure that the location of the library files are on the MATLAB path.']);
            
        end
        
    end
    
else
    
    error('PS5000Config:OperatingSystemNotSupported', 'Operating system not supported - please contact support@picotech.com');
    
end

%% Set Path for PicoScope Support Toolbox Files if Not Installed
% Set MATLAB Path to include location of PicoScope Support Toolbox
% Functions and Classes if the Toolbox has not been installed. Installation
% of the toolbox is only supported in MATLAB 2014b and later versions.

% Check if PicoScope Support Toolbox is installed - using code based on
% <http://stackoverflow.com/questions/6926021/how-to-check-if-matlab-toolbox-installed-in-matlab How to check if matlab toolbox installed in matlab>

ps5000ConfigInfo.psTbxName = 'PicoScope Support Toolbox';
ps5000ConfigInfo.v = ver; % Find installed toolbox information

if(~any(strcmp(ps5000ConfigInfo.psTbxName, {ps5000ConfigInfo.v.Name})))
   
    warning('PS5000Config:PSTbxNotFound', 'PicoScope Support Toolbox not found, searching for folder.');
    
    % If the PicoScope Support Toolbox has not been installed, check to see
    % if the folder is on the MATLAB path, having been downloaded via zip
    % file or copied from the Microsoft Windows Pico SDK installer
    % directory.
    
    ps5000ConfigInfo.psTbxFound = strfind(path, ps5000ConfigInfo.psTbxName);
    
    if(isempty(ps5000ConfigInfo.psTbxFound) && ispc())
        
        % Check if the folder is present in the relevant SDK installation
        % directory on Windows platforms (if the SDK installer has been
        % used).
        
        % Obtain the folder name
        ps5000ConfigInfo.psTbxFolderName = fullfile(ps5000ConfigInfo.winSDKInstallPath, 'MATLAB' , ps5000ConfigInfo.psTbxName);

        % If it is present in the SDK directory, add the PicoScope Support
        % Toolbox folder and sub-folders to the MATLAB path.
        if(exist(ps5000ConfigInfo.psTbxFolderName, 'dir') == 7)

            addpath(genpath(ps5000ConfigInfo.psTbxFolderName));

        end
            
    else
        
        warning('PS5000Config:PSTbxDirNotFound', 'PicoScope Support Toolbox directory not found.');
            
    end
    
end

% Change back to the folder where the script was called from.
cd(ps5000ConfigInfo.workingDir);

%% Load Enumerations and Structure Information
% Enumerations and structures are used by certain shared library API functions.

% Find prototype file names based on architecture

ps5000ConfigInfo.ps5000MFile = str2func(strcat('ps5000MFile_', ps5000ConfigInfo.archStr));
ps5000ConfigInfo.ps5000WrapMFile = str2func(strcat('ps5000WrapMFile_', ps5000ConfigInfo.archStr));

[ps5000Methodinfo, ps5000Structs, ps5000Enuminfo, ps5000ThunkLibName] = ps5000ConfigInfo.ps5000MFile(); 

[ps5000WrapMethodinfo, ps5000WrapStructs, ps5000WrapEnuminfo, ps5000WrapThunkLibName] = ps5000ConfigInfo.ps5000WrapMFile();