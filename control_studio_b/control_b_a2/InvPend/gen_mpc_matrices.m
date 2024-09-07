%% MPC setting
% use same Q, R, and P from LQR

% Implement here an algorithm that allows you to compute all the required
% matrices automatically 

%Expanded weighting matrices

% preallocate size of matrix
QN= zeros(N*n,N*n);
% loop up to last grid
for i = 1:N-1
    QN(1+(i-1)*n:i*n,1+(i-1)*n:i*n) = Q;
end
% assign the last grid the proper value
QN(1+(N-1)*n:N*n,1+(N-1)*n:N*n) = P;

% preallocate
RN= zeros(N*m, N*m);
% create block diagonal
for i = 1:N
    RN(1+(i-1)*m:i*m,1+(i-1)*m:i*m) = R;
end


%Expanded system matrices 
Lambda=zeros(N*n,n);
for i = 1:N
    Lambda(1+(i-1)*n:i*n,1:n) = A^i;
end

Phi=zeros(N*n,N*m);
for i = 1:N
    Phi(1 +(i-1)*n:i*n,i) = B;
    for j = N:-1:i+1
        Phi(1+(j-1)*n:j*n,i) = A^(j-i)*B;
    end
end

%Cost function matrices: 
%W and F are correct provided that Phi, Lambda, QN,and RN are also correct
W=Phi'*QN*Phi+RN;
W=(W+W')/2; %to ensure symmetry, i.e., W=W'
F=Phi'*QN*Lambda;

%Bound Constraint
if (N<1)
    N=1;
end
Umax=[];
Umin=[];
Xmax=[];
Xmin=[];

for k=1:N %loop to form the 
    Umax=[Umax;umax]; 
    Umin=[Umin;umin];
    
    Xmax=[Xmax;xmax];
    Xmin=[Xmin;xmin];
end

%Inequality constraint  AN*U(k) < bN
% This matrix is correct, provided you have properly computed Phi
% Therefore, do not change it.
INm=eye(N*m);
AN=[INm;
   -INm;
    Phi;
   -Phi];

%% bN must be computed inside the controller

 
