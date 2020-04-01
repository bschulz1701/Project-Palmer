%Recorded Data
P = [983, 1020, 1100, 1150]; %pressure, mBar
V = [8.943, 8.97, 9.3, 9.5]; %Voltage, mV (terminated w/100ohm)

Per = (P./min(P))*21; %use ideal gas law to convert to percentage O2

Poly = polyfit(Per, V, 1); %Find linear fit values for mV/O2%
Per_Fit = linspace(10, 30, 100); %Generate %O2 sweep 
Fit = Poly(1)*Per_Fit + Poly(2); %Calculate mV output vals

close all
plot(Per, V); %Plot voltage output vs percentage O2
hold on
plot(Per_Fit, Fit); %Plot fit results

PolyV = polyfit(V, Per, 1); %Find linear fit values for %O2/mV
%Note: to calculate %O2 from mV
% Per O2 = PolyV(1)*V[mV] + PolyV(2)
disp(PolyV(1));
disp(PolyV(2));
