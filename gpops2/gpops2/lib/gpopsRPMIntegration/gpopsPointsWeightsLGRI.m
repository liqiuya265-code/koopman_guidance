function [tau, weights, E, F] = gpopsPointsWeightsLGRI(N)

% gpopsPointsWeightsLGRI
% this function finds the N Legendre-Gauss-Radau points, quadrature weights
% the LGR integration matrix E
% and the initial value matrix F
% E and F are stored as sparsely
% (row, colunm, value)

% initial guess for LGR nodes
Nm1 = N-1;
tau = -cos(2*pi*(0:Nm1)/(2*Nm1+1))';

% the Legendre Vandermonde matrix
P = zeros(N,N+1);
xold = 2;

% Newton Raphson method
Nindex = 2:N;
while max(abs(tau-xold))>eps
    xold = tau;
    P(1,:) = (-1).^(0:N);
    P(Nindex,1) = 1;
    P(Nindex,2) = tau(Nindex);
    for k = 2:N
        P(Nindex,k+1) = ( (2*k-1)*tau(Nindex).*P(Nindex,k)-(k-1)*P(Nindex,k-1) )/k;
    end
    tau(Nindex) = xold(Nindex)-((1-xold(Nindex))/N).*(P(Nindex,N)+P(Nindex,N+1))./(P(Nindex,N)-P(Nindex,N+1));
end

% the Legendre-Gauss-Radau Vandermonde
P = P(1:N,1:N);

% compute the weights
weights = zeros(N,1);
weights(1) = 2/N^2;
weights(Nindex) = (1-tau(Nindex))./(N*P(Nindex,N)).^2;

% compute differentiation matrix
xxPlusEnd = [tau; 1];
M = length(xxPlusEnd);
M1 = M+1;
M2 = M*M;

% compute the barycentric weights
Y = repmat(xxPlusEnd,1,M);
Ydiff = Y - Y'+eye(M);

WW = repmat(1./prod(Ydiff,2),1,M);
D = WW./(WW'.*Ydiff);

D(1:M1:M2) = 1-sum(D);

% full differentiation matrix
D = -D';
D = D(1:N,:);

% find integration matrix E
Emat = inv(D(:,2:N+1));

% find Ediag and Eoffdiag as a sparse triplet
% (row, colunm, value)
E = zeros(N.*N,3);

% seperate diagnal from off diagnal
index = 1:N;
for rowcount = 1:N;
    E(index,1) = rowcount.*ones(N,1);
    E(index,2) = (1:N)';
    E(index,3) = Emat(rowcount,:)';
    index = index + N;
end

% initial value matrix
F = [(1:N)', ones(N,1), ones(N,1)];

