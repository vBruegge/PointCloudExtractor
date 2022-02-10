in = readmatrix("Results/wing_110mm.dat");
%in_scan1 = readmatrix("Results/wing_110mm.dat_upper.txt");
%in_scan2 = readmatrix("Results/wing_110mm.dat_lower.txt");
org = readmatrix("SD7037.dat");
chord = 294.868073;%360.730499;..

in_scan = [in_scan1; in_scan2];
in_scan = in_scan/chord;
in_scan = in_scan + [0.5 0];

[tmp, indexMin] = min(in(:,1));
[tmp2, indexMinOrg] = min(org(:,1));
%inInterpBottom = interp1(in(1:indexMin,1),in(1:indexMin,2), org(1:indexMinOrg,1));
%inInterpTop = interp1(in(indexMin:end,1),in(indexMin:end,2), org(indexMinOrg:end,1));

errorBottom = (org(1:indexMinOrg,2) - inInterpBottom);
errorTop = (org(indexMinOrg:end,2) - inInterpTop);
meanError = (mean(abs(errorBottom))+mean(abs(errorTop)))/2;
maxError = max(max(abs(errorBottom)), max(abs(errorTop)));

fprintf("Average Error: %0.6f\n", meanError);
fprintf("Maximal Error: %0.6f\n", maxError);

figure(1)
plot(in(:,1), in(:,2))
hold on
plot(org(:,1), org(:,2))
plot(in_scan(:,1), in_scan(:,2))
xlabel('x/c')
ylabel('y/c')
legend('extracted airfoil', 'original airfoil', 'scan')

figure(2)
plot(org(1:indexMinOrg,1), abs(errorBottom))
hold on
plot(org(indexMinOrg:end,1), abs(errorTop))
xlabel('x/c')
ylabel('\Delta y/c')
legend('error top surface', 'error bottom surface')