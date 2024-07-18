function xi_hat_dot = observer_leader(ui,xi_hat,eps_i)

global A B c F L_obs

xi_hat_dot = zeros(2,1);

xi_hat_dot = A*xi_hat - L_obs*eps_i;   

xi_hat_dot = double(xi_hat_dot);
