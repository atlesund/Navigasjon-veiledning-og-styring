A = [ -0.1203 0.1479 -0.9860 0.0636 0
    -17.5184 -1.2523 0.5484 0 0
    3.5318 -0.0274 -0.1788 0 0
    0 1 0.1489 0 0 
    0 0 1 0 0];

Alat = A(1:4,1:4);
[V,D] = eig(Alat);

% Column V(:,k) belongs to eigenvalue D(k,k).
k = 4;                 % Choose one mode to investigate
    lambda = D(k,k);
v = V(:,k);

t = linspace(0, 50, 1000); % Choose a suitable time span
    x = real(v * exp(lambda*t));

tiledlayout(4,1)
names = {'Sideslip beta','Roll rate p','Yaw rate r','Bank angle phi'};

for n = 1:4
    nexttile
    plot(t,x(n,:))
    ylabel(names{n})
    grid on
end
xlabel('Time [s]')