% Static force analysis of a balloon rigged with 12 ground tethers (Design cliff2)
% By Thomas Bewley, JPL (on loan from UCSD)

% view([-111.6,8]); axis equal 
config=1; S1=3;   % Symmetry (3-fold)
switch config
    case 1
        Rb=80;          % Radius of balloon
        Rp=80;          % Radius of payload
        Hb=2200;        % Height of balloon
        Hp=1900;        % Height of payload 
        psi=-70*(pi/180);          % Extra rotation of payload & balloon
        off=[2200; -330];   % Center of payload / balloon system
        P(1:3,1)=[2061; -765.6; 1555];   % low   
        P(1:3,2)=[1344; -280.2; 1344];   % low   anchor points on ground
        P(1:3,3)=[2528; -166.2; 2077];   % high   

        
%        P(1:3,1)=[1594; -623.8; 1340];   % low   anchor points on ground
%        P(1:3,2)=[2299;  121.6; 2018];   % low
%        P(1:3,3)=[2785; -768.8; 1998];   % high  
    otherwise
        Rb=2;       % Radius of balloon 
        Rp=2;       % Radius of payload
        Hb=34;      % Height of balloon
        Hp=20;      % Height of payload
        Hcliff=30;  % Height of cliff
        psi=0;      % Extra rotation of payload & balloon
        off=[0; 0]; % Center of payload / balloon system
        Rg=20;      % Radius of ground attachment points
        phi=4*pi/3; P(1:3,1)=[Rg*sin(phi); Rg*cos(phi); 0];  % anchor points on ground
        phi=0;      P(1:3,2)=[Rg*sin(phi); Rg*cos(phi); 0];
        phi=2*pi/3; P(1:3,3)=[Rg*sin(phi); Rg*cos(phi); Hcliff];
end

% Test loads on the balloon:
lift=200;     % (lift force of balloon, in Newtons)
weight=100;   % (weight of payload, in Newtons)
Bdisturb=18.5 % (horizontal force on balloon, in Newtons)
Pdisturb=4.5    % (horizontal force on payload, in Newtons)

% Note: This system needs no addtional constraints and has no soft nodes.

% END OF ADJUSTABLE INPUTS

dim=3;       % dimension of system (2D or 3D)
b=2*S1;      % number of bars
s=4*S1+2;    % number of strings
m=b+s        % number of members
q=2*S1;      % number of free nodes Q
p=S1;        % number of fixed nodes P
n=q+p;       % number of nodes

% Now construct the 3-dimensional balloon configuration from above parameters
% Locations of free nodes are P=P_(dim x p) and fixed nodes are Q=Q_(dim x q)
for i=1:S1
    phi1=pi+2*pi*(i-1)/S1+psi; phi=2*pi*(i-1)/S1+psi;
    Q(1:3,     i)=[Rp*sin(phi)+off(1);  Rp*cos(phi)+off(2);  Hp]; % corners of payload
    Q(1:3,  S1+i)=[Rb*sin(phi1)+off(1); Rb*cos(phi1)+off(2); Hb]; % anchor points on balloon
end

C=zeros(m,n);                               % Connectivity matrix
for i=1:S1, j=mod(i,S1)+1; k=mod(i+1,S1)+1; 
  C(     i,   i)=1; C(     i,     j)=-1;    % bars modelling payload
  C(  S1+i,S1+i)=1; C(  S1+i,  S1+j)=-1;    % bars modelling balloon                                         
  C(2*S1+i,S1+i)=1; C(2*S1+i,     j)=-1;    % strings from balloon to payload (set 1)
  C(3*S1+i,S1+i)=1; C(3*S1+i,     k)=-1;    % strings from balloon to payload (set 2)
  C(4*S1+i,S1+i)=1; C(4*S1+i,2*S1+i)=-1;    % strings from balloon to ground (set 1)
  C(5*S1+i,S1+i)=1; C(5*S1+i,2*S1+k)=-1;    % strings from balloon to ground (set 2)
end
C(6*S1+1,   3)=1; C(6*S1+1,2*S1+1)=-1;    % strings from payload to ground
C(6*S1+2,   1)=1; C(6*S1+2,2*S1+2)=-1;    % strings from payload to ground
% C(6*S1+2,   2)=1; C(6*S1+2,2*S1+1)=-1;    % strings from payload to ground
% C(6*S1+3,   3)=1; C(6*S1+3,2*S1+1)=-1;    % strings from payload to ground
tensegrity_plot(Q,P,C,b,s); hold on;
[x,y,z] = ellipsoid(0,0,Hb,Rb,Rb,Rb*2/3,30); surf(x+off(1),y+off(2),z)

% Define applied external force U=U_(dim x p)
U(1,[1:S1])=0;      U(2,[1:S1])     =Pdisturb/S1; U(3,[1:S1])=-weight/S1;
U(1,[S1+1:2*S1])=0; U(2,[S1+1:2*S1])=Bdisturb/S1; U(3,[S1+1:2*S1])=lift/S1;

% Finally, solve for the forces at equilibrium.
if exist('constraints')
    [c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U,constraints);
else
    [c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
end

% end script 2D
