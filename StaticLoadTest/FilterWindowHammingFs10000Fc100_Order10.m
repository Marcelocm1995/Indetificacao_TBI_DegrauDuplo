function y = FilterWindowHammingFs10000Fc100_Order10(x)
%FILTERWINDOWHAMMINGFS10000FC100_ORDER10 Filters input x and returns output y.

% MATLAB Code
% Generated by MATLAB(R) 9.4 and DSP System Toolbox 9.6.
% Generated on: 31-Aug-2022 19:59:20

%#codegen

% To generate C/C++ code from this function use the codegen command. Type
% 'help codegen' for more information.

persistent Hd;

if isempty(Hd)
    
    % The following code was used to design the filter coefficients:
    % % FIR Window Lowpass filter designed using the FIR1 function.
    %
    % % All frequency values are in Hz.
    % Fs = 10000;  % Sampling Frequency
    %
    % N    = 10;       % Order
    % Fc   = 100;      % Cutoff Frequency
    % flag = 'scale';  % Sampling Flag
    %
    % % Create the window vector for the design algorithm.
    % win = hamming(N+1);
    %
    % % Calculate the coefficients using the FIR1 function.
    % b  = fir1(N, Fc/(Fs/2), 'low', win, flag);
    
    Hd = dsp.FIRFilter( ...
        'Numerator', [0.0144006686788034 0.0303952303143755 ...
        0.0723780841993072 0.124507368574912 0.166816732982725 0.183003830499755 ...
        0.166816732982725 0.124507368574912 0.0723780841993072 ...
        0.0303952303143755 0.0144006686788034]);
end

y = step(Hd,double(x));


% [EOF]