upper = readmatrix("Results/wing_110mm.dat_lower.txt");
degree = 8;
upper = upper - upper(1,:);
upper = upper/max(upper(:,1));
upper = upper - [0 -0.0059557];

binCoeff = zeros(degree+1,1);
for i = 0:degree
    binCoeff(i+1) = nchoosek(degree, i);
end
A = zeros(length(upper), degree);
for i = 0:degree
    for j=1:length(upper)
        A(j,i+1) = binCoeff(i+1)*(1-upper(j,1))^(degree+1-i)*upper(j,1)^i*sqrt(upper(j,1));
    end
end
coeff = A\upper(:,2);
fitted = zeros(length(upper),1);
for i=1:length(upper)
    for j=0:degree
        fitted(i) = fitted(i)+coeff(j+1)*binCoeff(j+1)*(1-upper(i,1))^(degree+1-j)*upper(i,1)^j*sqrt(upper(i,1));
    end
end
fitted2 = fitted + [0 -0.0059557];
upper = upper + [0 -0.0059557];
newUpper = readmatrix("Results/wing_110mm.dat_newLower.txt");
figure(3)
%plot(upper(:,1),upper(:,2))
hold on
plot(newUpper(:,1), newUpper(:,2))
%plot(upper(:,1), fitted)
%plot(upper(:,1), fitted2)