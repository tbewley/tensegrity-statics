% Static force analysis of a balloon rigged with 12 ground tethers (Design B)
% By Thomas Bewley, JPL (on loan from UCSD)
clear; clf; 
% Note: This system needs no addtional constraints and has no soft nodes.

dim=3;    % dimension of system
b=5;      % number of bars
s=9;      % number of strings
m=b+s;    % number of members
q=6;      % number of free nodes Q
p=0;      % number of fixed nodes P
n=q+p;    % number of nodes

% Now construct the 3-dimensional balloon configuration from above parameters
% Locations of free nodes are P=P_(dim x p) and fixed nodes are Q=Q_(dim x q)
r=0.5;
Q(:,1)=[0; 0;   0];
Q(:,2)=[1; 0;   0];
Q(:,3)=[2; 0;   0];
Q(:,4)=[1; r*cos(0);      r*sin(0)];
Q(:,5)=[1; r*cos(2*pi/3); r*sin(2*pi/3)]; 
Q(:,6)=[1; r*cos(4*pi/3); r*sin(4*pi/3)]; 
P=[];
C=zeros(m,n);                               % Connectivity matrix
C( 1,1)=1; C( 1,2)=-1;    % bars 
C( 2,2)=1; C( 2,3)=-1;
C( 3,2)=1; C( 3,4)=-1;
C( 4,2)=1; C( 4,5)=-1;
C( 5,2)=1; C( 5,6)=-1;
C( 6,1)=1; C( 6,4)=-1;    % strings 
C( 7,1)=1; C( 7,5)=-1;    
C( 8,1)=1; C( 8,6)=-1;    
C( 9,3)=1; C( 9,4)=-1;    
C(10,3)=1; C(10,5)=-1;    
C(11,3)=1; C(11,6)=-1;    
C(12,4)=1; C(12,5)=-1;    
C(13,5)=1; C(13,6)=-1;    
C(14,6)=1; C(14,4)=-1;    

% Define applied external force U=U_(dim x q)
U(1:dim,1:6)=0; U(1,1)=1; U(1,3)=-1;

% Finally, solve for the forces at equilibrium.
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
figure(2); tensegrity_plot(Q,P,C,b,s,U,V); axis equal

% end script 2D
