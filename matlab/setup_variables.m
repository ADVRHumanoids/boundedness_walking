A = [0 1; 10 0];
B = [0; -10];
lipm = ss(A, B, [1 0], 0);
Klipm = lqr(lipm, diag([1 1]), 1);
lipm = ss([A, [0; 0]; [0 1 0]], [B; 0], [1 0 0], 0);
lipm_d = c2d(lipm, 1.0);
K = dlqr(lipm_d.A, lipm_d.B, diag([0 1 1]), 0.0001);