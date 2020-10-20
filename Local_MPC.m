clear;clc;
% Define Scenario Variables
    syms x y theta v t x_d y_d theta_d v_d
    State_Variables = [x;y;theta;v];
    syms T Delta  T_d Delta_d
    Manipulated_Variables=[T;Delta];
    Ts=0.001;
    S_d=[x_d;y_d;theta_d;v_d];
    U_d=[T_d;Delta_d];
    x_d=v_d*t; y_d=0; theta_d=0; v_d=15;
    T_d=0; Delta_d=0;
    Length_of_vehicle=5;
    
% Define Model
    Prediction_Horizon=10;
    Control_Horizon=10;
    q_i=350;
    [S_X,S_U,S_D,q] = State_Estimator(State_Variables,Manipulated_Variables,Ts,S_d,U_d,Prediction_Horizon,Control_Horizon,Length_of_vehicle,q_i);
    plot_results(S_X,S_D,q)

% Function to calculate state by using sytem dynamics
function State = System_Dynamics(State_Variables,Manipulated_Variables,Length_of_Vehicle)
    % State_VariablesSt
        x=State_Variables(1); y=State_Variables(2); theta=State_Variables(3); v=State_Variables(4);
        
    % Manipulated_Variables
        T=Manipulated_Variables(1); Delta= Manipulated_Variables(2);
        
    % Length_of Vehicle
        Cl=Length_of_Vehicle;
        
    % State_Space Form
        x_dot=v*cos(theta);  y_dot=v*sin(theta); theta_dot=(tan(Delta)*v)/Cl; v_dot=0.5*T;
        
    % State_Space Form
        State=[x_dot;y_dot;theta_dot;v_dot];
end

% Function to linearise the system 
function [A,B,C,D] =Linearise(Length_of_vehicle,Ts,S_d,U_d)
    State = System_Dynamics(S_d,U_d,Length_of_vehicle);
    A=jacobian(State,S_d);
    B=jacobian(State,U_d);
    C=eye(4);
    D=zeros(size(B));
end

% Function to discretize the system 
function [Ad,Bd,Cd,Dd] =Discretize(A,B,C,D,Ts)
    nx=size(A,1);
    nu=size(B,2);
    M=expm([[A B]*Ts;zeros(nu,nx+nu)]);
    Ad=M(1:nx,1:nx);Bd=M(1:nx,nx+1:nx+nu);Cd=C;Dd=D;
end

% Function to calculate error model
function Error_Model=Error(S_d,U_d,State_Variables,Manipulated_Variables)
Error_Model(1:4,1)=State_Variables-S_d;
Error_Model(5:6,1)=Manipulated_Variables-U_d;
end

% Function to calculate the optimised state and control values
function [S_X,S_U,S_D,q]=State_Estimator(State_Variables,Manipulated_Variables,Ts,S_d,U_d,Prediction_Horizon,Control_Horizon,Length_of_vehicle,q_i)
   % Defining symbolic Variables  
       syms x y theta v x_d y_d theta_d v_d T_d Delta_d t
       
   % Linearising the system
       [A,B,C,D]=Linearise(Length_of_vehicle,Ts,S_d,U_d);
       
   % Discretising the system
       [Ad,Bd,Cd,Dd]=Discretize(A,B,C,D,Ts);
   % Building the Error Dynamics    
       Error_Model=Error(S_d,U_d,State_Variables,Manipulated_Variables);
       
   % Loading the reference trajectory
%        xr=load("x_1.mat").x;
%        yr=load("y_1.mat").y;
%        theta_r=load("theta_1.mat").theta;
%        vr=ones(length(xr),1)*15;
       V=5;
       Time=0:0.1:100;
       xr=V*cos(Time);
       yr=V*sin(Time);
       theta_r=Time;
       vr=ones(length(xr),1)*V;
    % Substituting the values   
        x=0;y=0;theta=pi/4;v=1;x_d=xr(1); y_d=yr(1); theta_d=theta_r(1); v_d=vr(1);T_d=0; Delta_d=0;
        T_d=0;
        Delta_d=atan2(V,Length_of_vehicle);
        A=double(subs(Ad));
        B=double(subs(Bd));
        Z=double(subs(Error_Model(1:4,1)));
        K= place(A,B,[-2,-0.1,-0.01,-0.01]); % Calculating K value for controller
        U=-K*Z; % Defining the control input
        T=U(1);Delta=U(2);
        
    % Initialising  the Output variables
        S_X=double(subs(State_Variables)); % Matrix to store the optimised trajectory values
        S_U=double(subs(Manipulated_Variables)); % Matrix to store the optimised control inputs
        S_D=double(subs(S_d)); % Matrix to store the desired trajectory values
        U_D=double(subs(U_d)); % Matrix to store the desired control inputs
   
    % Variables to stack A and B matrices
    Stack_A=A;
    Stack_B=B;
    Stack_C=eye(4);
    
    % Weights for cost function
        Qp = diag([1000 1000 10 10]); % Weight for error in trajectory
        Rp = 0.01*eye(2); % Weight for error in control input
        Qt = diag([10 10 100 100]); % Terminal weight for error in trajectory
        Rt = 0.1*eye(2); % Terminal weight for error in control input
    
    % Constraints
        U_max=ones(Control_Horizon*2,1)*100;
   
    % Looping over timesteps
    for q=0:(length(xr)-Prediction_Horizon-1)
        % Looping over Prediction Horizon
        for i_p=2+q:Prediction_Horizon+1+q   
            % Next State Estimation
                if i_p<=Control_Horizon+q
                    U_Calc=U(((i_p-2)*2)+1:(2*(i_p-1)),1);
                else
                    U_Calc=zeros(2,1);
                end
                Z(((i_p-1)*4)+1:(4*(i_p)),1)=A(((i_p-2)*4)+1:(4*(i_p-1)),1:4)*Z(((i_p-2)*4)+1:(4*(i_p-1)),1)+B(((i_p-2)*4)+1:(4*(i_p-1)),1:2)*U_Calc;
                
            %  Value Substitution in desired variables
                x_d=xr(i_p);y_d=yr(i_p); theta_d=theta_r(i_p); v_d=vr(i_p);T_d=0; Delta_d=0;
                S_D(((i_p-1)*4)+1:(4*(i_p)),1)=double(subs(S_d));
                U_D(((i_p-1)*2)+1:(2*(i_p)),1)=double(subs(U_d));
                
            % Estimation of A and B matrices
                A(((i_p-1)*4)+1:(4*(i_p)),1:4)=double(subs(Ad));
                B(((i_p-1)*4)+1:(4*(i_p)),1:2)=double(subs(Bd));
            
            % Calculation for control input for next step
                if i_p<=Control_Horizon+q
                    U(((i_p-1)*2)+1:(2*(i_p)),1)= (-1)*K*Z(((i_p-1)*4)+1:(4*(i_p)),1);
                end   
        end
        
       % Calculation of matrices for cost function
       for i=2:Prediction_Horizon
            Stack_A(((i-1)*4)+1:(4*(i)),1:4)=A(((i-1)*4)+1:(4*(i)),1:4)*Stack_A(((i-2)*4)+1:(4*(i-1)),1:4);
            Qp(((i-1)*4)+1:(4*(i)),((i-1)*4)+1:(4*(i)))=diag([100000 1000 100 1000]);
       end
       Qp(((Prediction_Horizon-1)*4)+1:(4*(Prediction_Horizon)),((Prediction_Horizon-1)*4)+1:(4*(Prediction_Horizon)))=Qt; 
       for j=1:Control_Horizon
            mat=B(((j-1)*4)+1:(4*(j)),1:2);
            Stack_C=eye(4);
            Stack_B(((j-1)*4)+1:(4*(j)),((j-1)*2)+1:(2*(j)))=mat;
            Rp(((j-1)*2)+1:(2*(j)),((j-1)*2)+1:(2*(j)))= 0.01*eye(2);
            for i=1:Prediction_Horizon-j
                Stack_C(((i)*4)+1:(4*(i+1)),1:4)=A(((i)*4)+1:(4*(i+1)),1:4)* Stack_C(((i-1)*4)+1:(4*(i)),1:4);
                Stack_B(4*(i+j-1)+1:4*(i+j),2*(j-1)+1:2*(j))=Stack_C(((i)*4)+1:(4*(i+1)),1:4)*mat;
            end
       end
        Rp(((Control_Horizon-1)*2)+1:(2*(Control_Horizon)),((Control_Horizon-1)*2)+1:(2*(Control_Horizon)))= Rt;
        
    % Calculating matrices for Cost Function
        H=2*(Stack_B.'*Qp*Stack_B+Rp);
        F=2*Stack_B.'*Qp*Stack_A*Z((q*1)*4+1:(q+1)*4,1);
        Cost=0.5*(U((q*1)*2+1:(q+Control_Horizon)*2,1).'*H*U((q*1)*2+1:(q+Control_Horizon)*2,1))+F.'*U((q*1)*2+1:(q+Control_Horizon)*2,1);
    %  Setting Constraints
    for i =1:1
        A_ineq(((i-1)*2*Control_Horizon)+1:((i)*2*Control_Horizon),1:2*Control_Horizon)=diag(((-1)^(i+1))*ones(2*Control_Horizon,1));
    end
    B_ineq=[U_max];
    A_eq=zeros(0,length(F));
    B_eq=zeros(0,1);
    opt = mpcActiveSetOptions;
    iA0=false(length(B_ineq),1);  
    if q>0
          iA0=true(length(B_ineq),1);
    end
    
    % Solving QP 
         [U_sol,exitflag] = mpcActiveSetSolver(H,F,A_ineq,B_ineq,A_eq,B_eq,iA0,opt);
     
    % Using the optimised values to calculate the variables
         U((q*1)*2+1:(q+Control_Horizon)*2,1)=U_sol;
         Z(((2+q-1)*4)+1:(4*(2+q)),1)=A(((2+q-2)*4)+1:(4*(2+q-1)),1:4)*Z(((2+q-2)*4)+1:(4*(2+q-1)),1)+B(((2+q-2)*4)+1:(4*(2+q-1)),1:2)*U(((2+q-2)*2)+1:(2*(2+q-1)),1);
         S_X(((2+q-1)*4)+1:(4*(2+q)),1)= Z(((2+q-1)*4)+1:(4*(2+q)),1)+S_D(((2+q-1)*4)+1:(4*(2+q)),1);
         S_U(((2+q-2)*2)+1:(2*(2+q-1)),1)=U(((2+q-2)*2)+1:(2*(2+q-1)),1)+ U_D(((2+q-2)*2)+1:(2*(2+q-1)),1); 
    disp(q);
    end
end
function plot_results(S_X,S_D,q)
    % Setting parameter matrices for plotting
         for i_p=1:q+1
             Q(i_p,1)=S_D((i_p-1)*4+1:(i_p-1)*4+1);
             Q(i_p,2)=S_D((i_p-1)*4+2:(i_p-1)*4+2);
             Q(i_p,3)=S_D((i_p-1)*4+3:(i_p-1)*4+3);
             Q(i_p,4)=S_D((i_p-1)*4+4:(i_p-1)*4+4);
             R(i_p,1)=S_X((i_p-1)*4+1:(i_p-1)*4+1);
             R(i_p,2)=S_X((i_p-1)*4+2:(i_p-1)*4+2);
             R(i_p,3)=S_X((i_p-1)*4+3:(i_p-1)*4+3);
             R(i_p,4)=S_X((i_p-1)*4+4:(i_p-1)*4+4);
         end
         % Plotting individual parameters
             figure('Name','X-Y-Theta-V Plot');
             % X-Plot
                 subplot(2,2,1)
                 plot(1:i_p,Q(1:i_p,1))
                 hold on
                 plot(1:i_p,R(1:i_p,1))
                 xlabel("Time Steps")
                 ylabel("X")
                 legend("X Desired","Actual X")
             % Y-Plot
                 subplot(2,2,2)
                 plot(1:i_p,Q(1:i_p,2))
                 hold on
                 plot(1:i_p,R(1:i_p,2))
                 xlabel("Time Steps")
                 ylabel("Y")
                 legend("Y Desired","Actual Y")
             % Theta-Plot
                 subplot(2,2,3)
                 plot(1:i_p,Q(1:i_p,3))
                 hold on
                 plot(1:i_p,R(1:i_p,3))
                 xlabel("Time Steps")
                 ylabel("Theta")
                 legend("Theta Desired","Actual Theta")
             % V-Plot
                 subplot(2,2,4)
                 plot(1:i_p,Q(1:i_p,4))
                 hold on
                 plot(1:i_p,R(1:i_p,4))
                 xlabel("Time Steps")
                 ylabel("V")
                 legend("V Desired","Actaul V")
                 
         % Plotting trajectories
             figure('Name','X-Y Plot')
             plot(Q(1:i_p,1),Q(1:i_p,2))
             hold on
             plot(R(1:i_p,1),R(1:i_p,2))
             xlabel("X")
             ylabel("Y")
             legend("Desired","Actual")
end


% function My_Cost_Function=Custom_Cost_Function()