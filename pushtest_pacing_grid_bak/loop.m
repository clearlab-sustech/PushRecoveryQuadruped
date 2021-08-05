clear all


% parameters
Tw = 0.6; %total time
f = 1/0.15; %frequency
Ns = Tw*f; %step number
T = 0.05; %discrete time
N = 1/(f*T); %steps of MPC
r = 0.01; %size of the input constraint for trot
theta=0; %inclination of slope
h=0.29*cos(-theta); %robot height
g=9.81;

% contact points
p1 = [0.19;-0.11];
p2 = [0.19;0.11];
p3 = [-0.19;-0.11];
p4 = [-0.19;0.11];

% trot
k1 = (p4(2)-p1(2))/(p4(1)-p1(1));
k2 = (p3(2)-p2(2))/(p3(1)-p2(1));
b1 = r;
b2 = -r;
b3 = r;
b4 = -r;

% system matrices
A=[0 1 0 0;
   g*cos(theta)/h 0 0 0;
   0 0 0 1;
   0 0 g*cos(theta)/h 0];
B=[0 0;-g*cos(theta)/h 0;0 0;0 -g*cos(theta)/h];
C=[0 1 0 0];
[AD,BD] = Continuous2Discrete(A,B,C,T);
model = LTISystem('A', AD, 'B', BD, 'Ts', T);


% state constraints
model.x.min = [-0.6; -2; -0.6; -2]; 
model.x.max = [0.6; 2; 0.5; 2];

%trot
for i=1:1:Ns
    % input constraints
    if mod(i,2)==0
        Aui = [1 0;-1 0;k2 -1;-k2 1];
        bui = [0.19;0.19;b3;-b4];
    else
        Aui = [1 0;-1 0;k1 -1;-k1 1];
        bui = [0.19;0.19;b1;-b2];
    end
    
   Uset = Polyhedron('A', Aui, 'b', bui);
   model.u.with('setConstraint');
   model.u.setConstraint = Uset;
   
   % terminal constraints
   if i==1
       Tset = Polyhedron( 'lb', [-0.19; 0; -0.11; 0], 'ub', [0.19; 0; 0.11; 0]);
   else
       At=F.A; 
       bt=F.b;
       Tset = Polyhedron( 'A', At, 'b', bt);
   end
   model.x.with('terminalSet');
   model.x.terminalSet = Tset;
   
   % cost function
   Q = eye(4);
   model.x.penalty = QuadFunction(Q);
   R = eye(2);
   model.u.penalty = QuadFunction(R);
   model.x.with('terminalPenalty');
   PN=10000*eye(4);
   model.x.terminalPenalty = QuadFunction(PN);
   ctrl = MPCController(model, N);
   explicit_ctrl = ctrl.toExplicit();
   F = explicit_ctrl.partition.Domain(); %feasible set
%    simple=explicit_ctrl.simplify;
end

% % crawl
% for i=1:1:Ns
%     % input constraints
%     ne12=norme(p1,p2);
%     ne24=norme(p2,p4);
%     ne41=norme(p4,p1);
%     ne32=norme(p3,p2);
%     ne43=norme(p4,p3);
%     ne23=norme(p2,p3);
%     ne31=norme(p3,p1);
%     ne14=norme(p1,p4);
%     if i==1
%         Aui=-[ne12(1) ne12(2);ne24(1) ne24(2);ne41(1) ne41(2)];
%         bui=[ne12(3);ne24(3);ne41(3)];
%     elseif i==2
%         Aui=-[ne24(1) ne24(2);ne32(1) ne32(2);ne43(1) ne43(2)];
%         bui=[ne24(3);ne32(3);ne43(3)];
%     elseif i==3
%         Aui=-[ne12(1) ne12(2);ne23(1) ne23(2);ne31(1) ne31(2)];
%         bui=[ne12(3);ne23(3);ne31(3)];
%     else
%         Aui=-[ne14(1) ne14(2);ne43(1) ne43(2);ne31(1) ne31(2)];
%         bui=[ne14(3);ne43(3);ne31(3)];
%     end
%     
%    Uset = Polyhedron('A', Aui, 'b', bui);
%    model.u.with('setConstraint');
%    model.u.setConstraint = Uset;
%    
%    % terminal constraints
%    if i==1
% %        Tset = Polyhedron( 'lb', [-0.19; 0; -0.11; 0], 'ub', [0.19; 0; 0.11; 0]);
%        At=[1 0 0 0;-1 0 0 0;0 0 1 0;0 0 -1 0];
%        bt=[0.19;0.19;0.11;0.11];
%        Ae=[0 1 0 0;0 0 0 1];
%        be=[0;0];
%        Tset = Polyhedron( 'A', At, 'b', bt, 'Ae', Ae, 'be', be);
%    else
%        At=F.A; 
%        bt=F.b;
%        Tset = Polyhedron( 'A', At, 'b', bt);
%    end
%    model.x.with('terminalSet');
%    model.x.terminalSet = Tset;
%    
%    % cost function
%    Q = eye(4);
%    model.x.penalty = QuadFunction(Q);
%    R = eye(2);
%    model.u.penalty = QuadFunction(R);
%    model.x.with('terminalPenalty');
%    PN=10000*eye(4);
%    model.x.terminalPenalty = QuadFunction(PN);
%    ctrl = MPCController(model, 3);
%    explicit_ctrl = ctrl.toExplicit();
%    F = explicit_ctrl.partition.Domain(); %feasible set
% %    simple=explicit_ctrl.simplify;
% end

function ne=norme(p1,p2)
ne=1/norm(p1-p2)*[p1(2)-p2(2);p2(1)-p1(1);p1(1)*p2(2)-p2(1)*p1(2)];
end