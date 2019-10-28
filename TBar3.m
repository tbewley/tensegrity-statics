% Static force analysis of 2D TBar (Fig 3.3 of Skelton & de Oliviera 2009)
% By Thomas Bewley, UC San Diego (+ faculty fellow at JPL)
clear; clf; figure(1);

% Free [Q=Q_(dim x q)] and fixed [Q=Q_(dim x q)] node locations
Q(:,1)=[0; 0; 0];
Q(:,2)=[1; 0; 0];
Q(:,3)=[2; 0; 0]; r=0.5;
Q(:,4)=[1; r*cos(0);      r*sin(0)];
Q(:,5)=[1; r*cos(2*pi/3); r*sin(2*pi/3)]; 
Q(:,6)=[1; r*cos(4*pi/3); r*sin(4*pi/3)]; P=[];
[dim,q]=size(Q); p=size(P,2); n=q+p; 

% Connectivity matrix
C(  1,1)=1; C(  1,2)=-1;       % bars 
C(  2,2)=1; C(  2,3)=-1;
C(  3,2)=1; C(  3,4)=-1;
C(  4,2)=1; C(  4,5)=-1;
C(  5,2)=1; C(  5,6)=-1; b=5;
C(b+1,1)=1; C(b+1,4)=-1;       % strings 
C(b+2,1)=1; C(b+2,5)=-1;    
C(b+3,1)=1; C(b+3,6)=-1;    
C(b+4,3)=1; C(b+4,4)=-1;    
C(b+5,3)=1; C(b+5,5)=-1;    
C(b+6,3)=1; C(b+6,6)=-1;    
C(b+7,4)=1; C(b+7,5)=-1;    
C(b+8,5)=1; C(b+8,6)=-1;    
C(b+9,6)=1; C(b+9,4)=-1; s=9; m=b+s;

% Applied external force U=U_(dim x q)
U(1:dim,1:6)=0; U(1,1)=1; U(1,3)=-1;

% Solve for the forces at equilibrium, and plot
[c_bars,t_strings,V]=tensegrity_statics(b,s,q,p,dim,Q,P,C,U);
tensegrity_plot(Q,P,C,b,s,U,V,true,1,0.08); grid on;

% end script TBar3
