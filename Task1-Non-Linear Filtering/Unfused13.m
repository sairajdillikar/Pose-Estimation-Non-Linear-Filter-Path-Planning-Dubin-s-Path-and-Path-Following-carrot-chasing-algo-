function u = Unfused13(u)
Inputs=load('my_input.mat');
Measurments=load('my_measurements.mat');

% Time vector
t = Inputs.t;
dt = t(2)-t(1); % Uniform 0.1s time vector

% Variance from assignment brief
SigmaSquare_v = 0.01; % (m/s)^2
SigmaSquare_Om = 0.25 ; % (m/s)^2
SigmaSquare_r1 = 0.01; % (m/s)^2
SigmaSquare_r2 = 0.09; % rad/s^s
SigmaSquare_b = 0.25; % rad/s^2

x = zeros(3,length(t));
x(:,1) = [1;1;0]; % Initial Robot Co-ordinates

Q= 0.02 * diag([0.01,0.25]); % Process Noise

% A: system transition matrix
A = [ 1 0 0;
      0 1 0;
      0 0 1];

L=Measurments.l;
h_nl = @(x) [sqrt((L(u,1)-x(1)).^2+(L(u,2)-x(2)).^2);
			   wrapToPi(atan2(L(u,2)-x(2),L(u,1)-x(1)))-x(3)];   

% P: State Covariance matrix (initial)
P = zeros(3,3,length(t));
P (:,:,1) = diag([1,1,1]) * 10^-5; 

% R: Measurement noise covariance matrix
RFu = diag([SigmaSquare_r1,SigmaSquare_b;]);%,SigmaSquare_r1,SigmaSquare_b,SigmaSquare_r1,SigmaSquare_b,SigmaSquare_r2,SigmaSquare_b,SigmaSquare_r2,SigmaSquare_b,SigmaSquare_r2,SigmaSquare_b;]); 


% EKF Filter algorithm
ChiStat = zeros(1,length(t));

for e = 1:length(t)-1
    z =[Measurments.r(e,u);Measurments.b(e,u)]; %Measurments.r(e,2);Measurments.b(e,2);Measurments.r(e,3);Measurments.b(e,3);Measurments.r(e,4);Measurments.b(e,4);Measurments.r(e,5);Measurments.b(e,5);Measurments.r(e,6);Measurments.b(e,6)];

    Gam = dt*[cos(x(3,e)) 0;
              sin(x(3,e)) 0;
              0 1 ];

    F = [1 0 -dt*(Inputs.v(1,e))*sin(x(3,e));
         0 1 dt*(Inputs.v(1,e))*cos(x(3,e));
         0 0 1];

    % Equation 1: State propagation
    xa = A*x(:,e) + Gam * [Inputs.v(1,e)+0.01; Inputs.om(1,e)+0.25];
    
    % Equation 2: State covariance propagation
    Pa = F*P(:,:,e)*F' + Gam*Q*Gam';

    % Measurement Update
    za = h_nl(xa); %Predicted measurements using a priori states
    
    % Jacobian matrix: linearization of the measurement model
    H = H_matrix(xa,Measurments.r(e,:),Measurments.b(e,:),L,u);   
    
    % Equation 4: Innovation covariance  
    S = H*Pa*H'+RFu;  
    Yzz = S^(-1);     %Information matrix of innovations
    
    % Equation 5: Kalman gain
    K = Pa*H'*Yzz; 
   
    % Equation 6: State update
    dz = z-za; %Innovations (using full non-linear model)
    x(:,e+1) = xa + K*dz; %State update
    
    % Equation 7: Error covariance update
    P(:,:,e+1) = Pa - (K * H * Pa);
    
    % Compute Chi-squared statistics:
    ChiStat(e+1) = dz'*Yzz*dz;
end

%Compute the filtered measurements (using filtered states and the full non-linear
%measurement model):
z_post = h_nl(xa);

figure(1);
plot(x(1,:),x(2,:))
hold on
% scatter(L(:,1),L(:,2),'r')

figure(2);
plot(t,ChiStat)
xlabel('Time [s]')
ylabel('\chi^2')
title('\chi^2 statistics about innovations')
end
