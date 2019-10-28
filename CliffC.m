% Static force analysis of a balloon rigged with 12 ground tethers
% Design cliff3: 4-fold symmetry, 2 anchor points above cliff, 2 below
% By Thomas Bewley, JPL (on loan from UCSD)

config=1;
switch config
    case 1
        Rb=80;          % Radius of balloon
        Rp=80;          % Radius of payload
        Hb=2200;        % Height of balloon
        Hp=1900;        % Height of payload 
        psi=-70*(pi/180);   % Extra rotation of payload & balloon
        off=[2200; -330];   % Center of payload / balloon system
        P(1:3,1)=[2061; -765.6; 1555];   % low   anchor points on ground
        P(1:3,2)=[1344; -280.2; 1344];   % low   
        P(1:3,3)=[2299;  121.6; 2018];   % high
        P(1:3,4)=[2785; -768.8; 1998];   % high
    otherwise
        Rb=2;       % Radius of balloon 
        Rp=2;       % Radius of payload
        Hb=34;      % Height of balloon
        Hp=20;      % Height of payload
        Hcliff=30;  % Height of cliff
        psi=0;      % Extra rotation of payload & balloon
        off=[0; 0]; % Center of payload / balloon system
        Rg=20;      % Radius of ground attachment points
        phi=0;      P(1:3,1)=[Rg*sin(phi); Rg*cos(phi); 0];  % anchor points on ground
        phi=pi/2;   P(1:3,2)=[Rg*sin(phi); Rg*cos(phi); 0];
        phi=pi;     P(1:3,3)=[Rg*sin(phi); Rg*cos(phi); Hcliff];
        phi=3*pi/2; P(1:3,4)=[Rg*sin(phi); Rg*cos(phi); Hcliff];
end

% Test loads on the balloon:
lift=200;     % (lift force of balloon, in Newtons)
weight=100;   % (weight of payload, in Newtons)
Bdisturb=18.5 % (horizontal force on balloon, in Newtons)
Pdisturb=4.5    % (horizontal force on payload, in Newtons)

% Note: This system needs no addtional constraints and has no soft nodes.

% END OF ADJUSTABLE INPUTS

dim=3;      % dimension of system (2D or 3D)
b=12;       % number of bars
s=18;       % number of strings
m=b+s       % number of members
q=8;        % number of free nodes Q
p=4;        % number of fixed nodes P
n=q+p;      % number of nodes

% Now construct the 3-dimensional balloon configuration from above parameters
% Locations of free nodes are P=P_(dim x p) and fixed nodes are Q=Q_(dim x q)
for i=1:4
    phi1=pi+pi/4+pi*(i-1)/2+psi; phi=pi+pi*(i-1)/2+psi;
    Q(1:3,    i)=[Rp*sin(phi)+off(1);  Rp*cos(phi)+off(2);  Hp]; % corners of payload
    Q(1:3,  4+i)=[Rb*sin(phi1)+off(1); Rb*cos(phi1)+off(2); Hb]; % anchor points on balloon
end

C=zeros(m,n);             % Connectivity matrix
C(1, 1)=1; C(1, 2)=-1;    % bars modelling payload
C(2, 2)=1; C(2, 3)=-1;    
C(3, 3)=1; C(3, 4)=-1;    
C(4, 4)=1; C(4, 1)=-1;    
C(5, 1)=1; C(5, 3)=-1;    
C(6, 2)=1; C(6, 4)=-1;    
C(7, 5)=1; C(7, 6)=-1;    % bars modelling balloon  
C(8, 6)=1; C(8, 7)=-1;
C(9, 7)=1; C(9, 8)=-1;
C(10,8)=1; C(10,5)=-1;
C(11,5)=1; C(11,7)=-1;
C(12,6)=1; C(12,8)=-1;

for i=1:4, j=mod(i,4)+1; k=mod(i-2,4)+1; 
  C(12+i,4+i)=1; C(12+i,i)=-1;    % strings from balloon to payload (set 1)
  C(16+i,4+i)=1; C(16+i,j)=-1;    % strings from balloon to payload (set 2)
  C(20+i,4+i)=1; C(20+i,8+i)=-1;  % strings from balloon to ground (set 1)
  C(24+i,4+i)=1; C(24+i,8+k)=-1;  % strings from balloon to ground (set 2)
end
C(29,2)=1; C(29,8+1)=-1;   % strings from payload to ground
C(30,3)=1; C(30,8+2)=-1;   % strings from payload to ground
tensegrity_plot(Q,P,C,b,s); hold on;
[x,y,z] = ellipsoid(0,0,Hb,Rb,Rb,Rb*2/3,30); surf(x+off(1),y+off(2),z)

% Define applied external force U=U_(dim x p)
U(1,[1:4])=0; U(2,[1:4])=Pdisturb/4; U(3,[1:4])=-weight/4;
U(1,[5:8])=0; U(2,[5:8])=Bdisturb/4; U(3,[5:8])=lift/4;

% Finally, solve for the forces at equilibrium.
if exist('constraints')
    [c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U,constraints);
else
    [c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
end

% end script 2D
