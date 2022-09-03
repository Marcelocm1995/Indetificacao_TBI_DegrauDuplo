function y = FilterWindowHammingFs200Fc10_Order1(x)
%FILTERWINDOWHAMMINGFS200FC10_ORDER1 Filters input x and returns output y.

% MATLAB Code
% Generated by MATLAB(R) 9.4 and DSP System Toolbox 9.6.
% Generated on: 31-Aug-2022 19:54:22

%#codegen

% To generate C/C++ code from this function use the codegen command. Type
% 'help codegen' for more information.

persistent Hd;

if isempty(Hd)
    
    % The following code was used to design the filter coefficients:
    % % FIR Window Lowpass filter designed using the FIR1 function.
    %
    % % All frequency values are in Hz.
    % Fs = 200;  % Sampling Frequency
    %
    % N    = 1;        % Order
    % Fc   = 10;       % Cutoff Frequency
    % flag = 'scale';  % Sampling Flag
    %
    % % Create the window vector for the design algorithm.
    % win = hamming(N+1);
    %
    % % Calculate the coefficients using the FIR1 function.
    % b  = fir1(N, Fc/(Fs/2), 'low', win, flag);
    
    Hd = dsp.FIRFilter();
end

y = step(Hd,double(x));


% [EOF]
