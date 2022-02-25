in = readmatrix("Results/wing_100mm.dat");
org = readmatrix("SD7037.dat");
chord = 294.868073;%360.730499;..
org = org(2:end-1,:);

[tmp, indexMin] = min(in(:,1));
[tmp2, indexMinOrg] = min(org(:,1));
indices = [99 103];
in(indices,:) = [];
inInterpBottom = interp1(in(1:indexMin-1,1),in(1:indexMin-1,2), org(1:indexMinOrg,1));
inInterpTop = interp1(in(indexMin:end,1),in(indexMin:end,2), org(indexMinOrg:end,1));
d = gradient(in(:,2), in(:,1));


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
plot(in(:,1), d)
xlabel('x/c')
ylabel('y/c')
legend('extracted airfoil', 'original airfoil')

figure(2)
plot(org(1:indexMinOrg,1), abs(errorBottom))
hold on
plot(org(indexMinOrg:end,1), abs(errorTop))
xlabel('x/c')
ylabel('\Delta y/c')
legend('error top surface', 'error bottom surface')