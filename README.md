# Dynamics_502
To run the code, please download the file and unzip it into a matlab directory. 
The only file that needs to be open to verify the results is main. This function calls all required files. 

The PDControllerODE.m file contains the ode45 function with the dynamical model, and the odesetpoint.m containts the ode45 function for the setpoint controller. 
The DynamicalModel.m function contains the derivation of a generalized dynamical model, and the M, C, and N matrices can be obtained by running the DynamicModelVariables.m file. 
The current M,N, and C matrices used can be seen by opening the M_N_C_variables.mat file. 