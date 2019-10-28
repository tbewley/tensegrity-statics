% Static force analysis of a balloon rigged with 9 ground tethers (Design D)
% By Thomas Bewley, JPL (on loan from UCSD)

clear; figure(1); clf;
Rb=2;     % Radius of balloon
Rp=1;     % Radius of payload
Rg=20;    % Radius of outer-most ground attachment points
Hb=36;    % Height of balloon
Hp=30;    % Height of payload (specified)
Hc=32.5;  % Height of convergence point

% Parameters for the 3-dimensional balloon configuration
S1=3;     % Symmetry of the substructure   (3-fold or 4-fold)
S2=3;     % Symmetry of the superstructure (3-fold or 4-fold)
GP=2;     % Number of ground points used per balloon tie-down point (1 or 2)
config=2; % Configuration for GP=2 ground points for balloons (1 or 2)
alpha=2*pi/3; % Opening angle for config=2 (GP=2) ground points for balloons

% Below are a few parameters specific to the balloon problem:
lift=200;   % (lift force of balloon, in Newtons)
weight=100; % (weight of payload, in Newtons)
Bdisturb=0; % (horizontal force on balloon, in Newtons)
Pdisturb=0; % (horizontal force on payload, in Newtons)

% Note: This system needs 1 addtional constraint and has no soft nodes.
extra_constraints=',x19==x20,x20==x21'

% END OF ADJUSTABLE INPUTS

dim=3;            % dimension of system
b=S1+S2;          % number of bars
s=2*S1+S2*(GP+1); % number of strings
m=b+s             % number of members
q=S1+S2+1;        % number of free nodes Q
p=S1+S2*GP;       % number of fixed nodes P
n=q+p;            % number of nodes

% Now construct the 3-dimensional balloon configuration from above parameters
% Locations of free nodes are P=P_(dim x p) and fixed nodes are Q=Q_(dim x q)
for i=1:S1                                   % SUBSTRUCTURE
    phi=pi+2*pi*(i-1)/S1;
    Q(1:3,i)=[Rp*sin(phi); Rp*cos(phi); Hp]; % corners of payload
    P(1:3,i)=[Rp*sin(phi); Rp*cos(phi); 0 ]; % ground points for payload
end
for i=1:S2                                   % SUPERSTRUCTURE
    phi=2*pi*(i-1)/S2; if config==1, offset=0; else, offset=alpha/2; end
    Q(1:3,S1+i)=[Rb*sin(phi);        Rb*cos(phi);        Hb]; % tie-off points on balloon
    P(1:3,S1+i)=[Rg*sin(phi+offset); Rg*cos(phi+offset); 0 ]; % outer ground points
    if GP==2                                                  % extra ground points
        if config==1, offset1=0; Rg1=Rb; else, offset1=-alpha/2; Rg1=Rg; end
        P(1:3,S1+S2+i)=[Rg1*sin(phi+offset1); Rg1*cos(phi+offset1); 0];
    end
end
Q(1:3,S1+S2+1)=[0; 0; Hc]; % Convergence point
C=zeros(m,n);              % Connectivity matrix
for i=1:S1,         j=mod(i,S2)+1;         % BARS:
  C(i,i)=1;         C(i,j)=-1;             % bars 1: modelling payload
end
for i=1:S2          j=S1+mod(i,S2)+1;
  C(S1+i,S1+i)=1;   C(S1+i,j)=-1;          % bars modelling balloon
end
for i=1:S1                                       % STRINGS:
  C(2*S2+i,i)=1;    C(2*S2+i,q+i)=-1;            % from payload to ground
  C(2*S2+S1+i,S1+S2+1)=1;  C(2*S2+S1+i,i)=-1;    % from convergence point to payload 
end
for i=1:S2
  C(3*S2+S1+i,S1+i)=1; C(3*S2+S1+i,S1+S2+1)=-1;  % from balloon to convergence point
  C(4*S2+S1+i,S1+i)=1; C(4*S2+S1+i,q+S1+i)=-1;   % 1st set from balloon to ground
  if GP==2, 
  C(5*S2+S1+i,S1+i)=1; C(5*S2+S1+i,q+2*S2+i)=-1; % 2nd set from balloon to ground
  end
end 

% Define applied external force U=U_(dim x p)
U(1,[1:S1])=0;        U(2,[1:S1])=Pdisturb/S1;        U(3,[1:S1])=-weight/S1;
U(1,[S1+1:S1+S2])=0;  U(2,[S1+1:S1+S2])=Bdisturb/S2;  U(3,[S1+1:S1+S2])=lift/S2;
U([1:3],S1+S2+1)=0;

% Finally, solve for the forces at equilibrium.
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U,extra_constraints);
tensegrity_plot(Q,P,C,b,s,U,V,false,1,4); grid on

% end script DesignD
