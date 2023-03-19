%An object travelling in 1D at a constant velocity of 1.5m/s
yk = zeros (1,200);
for n=1:200yk
 yk (n) = 1.5*n + 3*randn ;
end % creates 'measured' inputs with 'measurements' being independent
 %of each other i.e. erros don't propagate
R=1; %the function 'randn' ouputs normally distributed random numbers
 %this makes the standard deviation=1, therefore variance=1

X0=0; %starting at origin
P0=1; %any non-zero value otherwise K=0
A=1;
Q=0;
U=1.5; %travelling speed
W=0; %Assuming no white noise
H=1; %1 as just numbers not matrices
B = zeros (1,200);
for n=1:200
 B(n)= n;
end %for elapsed time
xkp = zeros (1,200);
x = zeros (1,200);
k = zeros (1,200);
pkp = zeros (1,200);
pk = zeros (1,200);
%t1 Predicted state
xkp(1)= A*X0 + B(1)*U + W;
pkp(1)= A*P0*A + Q;
%update w/ new measurements and kalman gain
k(1)=(pkp(1)*H)*inv(H*pkp(1)*H + R);
x(1)= xkp(1) + k(1)*(yk(1)-H*xkp(1));
pk(1)= (1-k(1)*H)*pkp(1);
for t=2:200
 %t(n) Predicted state
 xkp(t)= A*x(t-1) + 1*U + W;
 pkp(t)= A*pk(t-1)*A + Q;
 %update w/ new measurements and kalman gain
 k(t)=(pkp(t)*H)*inv(H*pkp(t)*H + R);
 x(t)= xkp(t) + k(t)*(yk(t)-H*xkp(t));
 pk(t)= (1-k(t-1)*H)*pkp(t-1);
end
test = linspace(0,300,200);
subplot(121)
plot(x)
hold on
plot (yk, 'Color','r')
subplot(122)
plot(x-test)
hold on
plot(yk-test, 'Color','r')
