
clc;
clear;
close all;

% =========================================================
% 1. INPUT SIGNAL GENERATION
% =========================================================

NUM_SAMPLES = 256;
Fs = 1000;

A1 = 1100;
f1 = 20;

A2 = 1000;
f2 = 180;

DC = 2000;

n = (0:NUM_SAMPLES-1)';
t = n/Fs;

x = DC + A1*sin(2*pi*f1*t) ...
       + A2*sin(2*pi*f2*t);

samples = round(x);
samples = double(samples(:));

if any(samples < -32768) || any(samples > 32767)
    error('Input exceeds signed 16-bit range.');
end

fprintf('\nGenerated %d input samples.\n', NUM_SAMPLES);


% =========================================================
% 2. FIR COEFFICIENT GENERATION
% =========================================================

NUM_TAPS = 4;
Fc = 100;

B = 16;
Q = 2^(B-1);

if NUM_TAPS < 2 || NUM_TAPS ~= floor(NUM_TAPS)
    error('Invalid number of FIR taps.');
end

if Fc <= 0 || Fc >= Fs/2
    error('Invalid cutoff frequency.');
end

h = zeros(1, NUM_TAPS);

M = (NUM_TAPS-1)/2;

for k = 0:NUM_TAPS-1

    if k == M
        hd = 2*Fc/Fs;
    else
        hd = sin(2*pi*Fc/Fs*(k-M)) ...
             /(pi*(k-M));
    end

    w = 0.54 - 0.46*cos(2*pi*k/(NUM_TAPS-1));

    h(k+1) = hd*w;

end

h = h/sum(h);

h_fixed = round(h*Q);

if any(h_fixed < -32768) || any(h_fixed > 32767)
    error('FIR coefficient exceeds signed 16-bit range.');
end

fprintf('\nFloating-point coefficients:\n');
disp(h);

fprintf('Q15 coefficients:\n');
disp(h_fixed);


% =========================================================
% 3. MATLAB REFERENCE FIR CALCULATION
% =========================================================

matlab_float_output = zeros(NUM_SAMPLES,1);
matlab_fixed_output = zeros(NUM_SAMPLES,1);

x_history = zeros(NUM_TAPS,1);

for k = 1:NUM_SAMPLES

    x_history = [samples(k); ...
                 x_history(1:NUM_TAPS-1)];

    acc_float = 0;
    acc_fixed = 0;

    for j = 1:NUM_TAPS

        acc_float = acc_float + ...
                    h(j)*x_history(j);

        acc_fixed = acc_fixed + ...
                    h_fixed(j)*x_history(j);

    end

    matlab_float_output(k) = acc_float;

    % Q15 scaling
    temp = floor(acc_fixed/Q);

    % Signed 16-bit wrap-around
    temp = mod(temp + 32768, 65536) - 32768;

    matlab_fixed_output(k) = temp;

end

fprintf('\nMATLAB FIR reference completed.\n');


% =========================================================
% 4. UART COMMUNICATION
% =========================================================

portName = 'COM3';
baudRate = 115200;
UART_TIMEOUT = 5;

fpga_output = zeros(NUM_SAMPLES,1);

s = serial(portName, ...
    'BaudRate', baudRate, ...
    'DataBits', 8, ...
    'Parity', 'none', ...
    'StopBits', 1, ...
    'FlowControl', 'none', ...
    'Timeout', UART_TIMEOUT, ...
    'InputBufferSize', 4096);

try

    fopen(s);
    flushinput(s);

    fprintf('\nUART connected: %s\n', portName);

    % -----------------------------------------------------
    % 4A. SEND COEFFICIENTS
    % -----------------------------------------------------

    fprintf('\nSending %d FIR coefficients...\n', NUM_TAPS);

    for k = 1:NUM_TAPS

        word = mod(h_fixed(k), 65536);

        lowByte = mod(word, 256);
        highByte = floor(word/256);

        fwrite(s, uint8([lowByte; highByte]), 'uint8');

        fprintf('Coefficient %d sent: %d\n', ...
                k-1, h_fixed(k));

    end

    fprintf('All coefficients transmitted.\n');


    % -----------------------------------------------------
    % 4B. SEND INPUT SAMPLES AND RECEIVE OUTPUT
    % -----------------------------------------------------

    fprintf('\nSending input samples to FPGA...\n');

    for k = 1:NUM_SAMPLES

        % Convert signed sample to 16-bit word
        word = mod(samples(k), 65536);

        lowByte = mod(word, 256);
        highByte = floor(word/256);

        % Transmit low byte first
        fwrite(s, uint8([lowByte; highByte]), 'uint8');

        fprintf('Sending sample %d: %d\n', k, samples(k));

        % Wait until two response bytes are available
        start_time = tic;

        while s.BytesAvailable < 2

            if toc(start_time) >= UART_TIMEOUT
                error('UART timeout at sample %d. Received only %d bytes.', ...
                      k, s.BytesAvailable);
            end

            pause(0.001);

        end

        % Read exactly two bytes
        response = fread(s, 2, 'uint8');

        if numel(response) ~= 2
            error('Incomplete FPGA response at sample %d.', k);
        end

        % Reconstruct 16-bit word
        result = double(response(1)) + ...
                 256*double(response(2));

        % Convert unsigned to signed
        if result >= 32768
            result = result - 65536;
        end

        fpga_output(k) = result;

        if mod(k,32) == 0 || k == NUM_SAMPLES

            fprintf('Received %d / %d samples\n', ...
                    k, NUM_SAMPLES);

        end

    end

    fclose(s);

    fprintf('\nUART communication completed.\n');

catch ME

    if strcmp(get(s,'Status'),'open')
        fclose(s);
    end

    delete(s);
    clear s;

    rethrow(ME);

end

delete(s);
clear s;


% =========================================================
% 5. COMPARE MATLAB AND FPGA OUTPUTS
% =========================================================

error_signal = matlab_fixed_output - fpga_output;

absolute_error = abs(error_signal);

mismatch_count = sum(error_signal ~= 0);

max_error = max(absolute_error);

mean_error = mean(absolute_error);

fprintf('\n========== FIR COMPARISON RESULTS ==========\n');

fprintf('Number of samples      : %d\n', NUM_SAMPLES);
fprintf('Number of FIR taps     : %d\n', NUM_TAPS);
fprintf('Maximum absolute error : %g\n', max_error);
fprintf('Mean absolute error    : %g\n', mean_error);
fprintf('Mismatched samples     : %d\n', mismatch_count);

if mismatch_count == 0
    fprintf('RESULT: FPGA MATCHES MATLAB!\n');
else
    fprintf('RESULT: OUTPUT MISMATCH DETECTED.\n');
end


% =========================================================
% 6. GENERATE EXCEL REPORT
% =========================================================

sample_index = (0:NUM_SAMPLES-1)';

result_table = [sample_index, ...
                samples, ...
                matlab_float_output, ...
                matlab_fixed_output, ...
                fpga_output, ...
                error_signal, ...
                absolute_error];

result_header = { ...
    'Sample_Index', ...
    'Input_Sample', ...
    'MATLAB_Float_Output', ...
    'MATLAB_Fixed_Output', ...
    'FPGA_Output', ...
    'Error', ...
    'Absolute_Error'};

coeff_table = [(0:NUM_TAPS-1)', ...
               h(:), ...
               h_fixed(:)];

coeff_header = { ...
    'Tap_Index', ...
    'Floating_Point_Coefficient', ...
    'Fixed_Point_Q15'};

summary_header = {'Parameter','Value'};

summary_data = { ...
    'Sampling_Frequency_Hz', Fs; ...
    'Cutoff_Frequency_Hz', Fc; ...
    'Number_of_Taps', NUM_TAPS; ...
    'Coefficient_Width', B; ...
    'Coefficient_Format', 'Q15'; ...
    'Number_of_Samples', NUM_SAMPLES; ...
    'Maximum_Absolute_Error', max_error; ...
    'Mean_Absolute_Error', mean_error; ...
    'Mismatched_Samples', mismatch_count};

excel_file = 'FIR_FPGA_Comparison.xlsx';

try

    xlswrite(excel_file, result_header, 'Results', 'A1');
    xlswrite(excel_file, result_table, 'Results', 'A2');

    xlswrite(excel_file, coeff_header, 'Coefficients', 'A1');
    xlswrite(excel_file, coeff_table, 'Coefficients', 'A2');

    xlswrite(excel_file, summary_header, 'Summary', 'A1');
    xlswrite(excel_file, summary_data, 'Summary', 'A2');

    fprintf('\nExcel report generated: %s\n', excel_file);

catch

    warning('Excel export failed. Creating CSV files.');

    csvwrite('FIR_FPGA_Results.csv', result_table);
    csvwrite('FIR_Coefficients.csv', coeff_table);

end


% =========================================================
% 7. PLOT INPUT AND OUTPUT
% =========================================================

figure;

subplot(3,1,1);

plot(t, samples);
title('Original Input Signal');
xlabel('Time (seconds)');
ylabel('Amplitude');
grid on;

subplot(3,1,2);

plot(t, matlab_fixed_output, 'b');
hold on;
plot(t, fpga_output, 'r--');

title('MATLAB Fixed-Point vs FPGA FIR Output');
xlabel('Time (seconds)');
ylabel('Amplitude');

legend('MATLAB Reference','FPGA Output');
grid on;

subplot(3,1,3);

plot(t, error_signal);

title('FPGA vs MATLAB Error');
xlabel('Time (seconds)');
ylabel('Error');
grid on;

fprintf('\nComplete FIR experiment finished!\n');
