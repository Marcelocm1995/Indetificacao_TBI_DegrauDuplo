function y = FilterWindowHammingFs1000Fc100_Order10(x)
%FILTER WINDOW HAMMING FS1000 FC100 Filters input x and returns output y.

% MATLAB Code
% Generated by MATLAB(R) 9.4 and DSP System Toolbox 9.6.
% Generated on: 21-Jul-2022 19:22:53

%#codegen

% To generate C/C++ code from this function use the codegen command. Type
% 'help codegen' for more information.

persistent Hd;

if isempty(Hd)
    
    % The following code was used to design the filter coefficients:
    % % FIR Window Lowpass filter designed using the FIR1 function.
    %
    % % All frequency values are in Hz.
    % Fs = 1000;  % Sampling Frequency
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
        'Numerator', [7.39142315096373e-19 0.00930428314501815 ...
        0.0475777661344174 0.122363546361145 0.20224655842984 0.237015691859159 ...
        0.20224655842984 0.122363546361145 0.0475777661344174 ...
        0.00930428314501815 7.39142315096373e-19]);
end

y = step(Hd,double(x));


% [EOF]
