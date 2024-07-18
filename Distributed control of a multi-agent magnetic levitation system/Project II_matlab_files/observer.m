function xi_hat_dot = observer(ui,xi_hat,eps_i)

global A B c F

% eps_i = eps_i + randn(1);

xi_hat_dot = zeros(2,1);

xi_hat_dot = A*xi_hat + B*ui - c*F*eps_i;   

xi_hat_dot = double(xi_hat_dot);
