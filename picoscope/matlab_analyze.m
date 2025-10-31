%% parameters of device unter test
P = PulseTest30VSource10;
 
t =  P(4:end, 3);
u =  P(4:end, 4);
i = -P(4:end, 5);
 
%% time-signal (excitation)
fs = 1 / mean(diff(t));
N  = length(t);
 
%% fft
fU = fftshift(fft(u));
fI = fftshift(fft(i));
 
f     = fs / N * ((-N/2):(N/2-1))';
omega = 2 * pi * f;
 
%% estimate parameters
idx = ((N/2+2):N)';
 
A = [fI(idx), (-1i ./ omega(idx)) .* fI(idx)];
b = fU(idx);
 
% solve linear system of equations A * x = b
x = A \ b;
 
%% extract estimated data
 
Rest = real(x(1));
Cest = real(1 / x(2));