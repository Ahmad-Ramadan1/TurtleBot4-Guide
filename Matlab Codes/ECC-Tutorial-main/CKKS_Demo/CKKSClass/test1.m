clear all
folderPaths = genpath('CKKSClass\');
addpath(folderPaths);

digits(100)         
s = vpa(10)^4;      % scaling factor
q = vpa(10)^15;     % base modulus
L = 2;              % number of levels
N = 2;              % key dimension
setup = Setup(N,q,s,L);
[pk,sk,setup] = KeyGen(setup);

% Encrypt the controller gains
K = 3;
K_ecd = Ecd(setup, K, 'type', 'scalar');
K_cyph = Enc(setup, K_ecd, pk);

L = 4;
L_ecd = Ecd(setup, L, 'type', 'scalar');
L_cyph = Enc(setup, L_ecd, pk);

J = K_cyph + L_cyph;
M = K_cyph * L_cyph;

J_dec = Dec(J, sk);
J_dcd = Dcd(J_dec);

M_dec = Dec(M, sk);
M_dcd = Dcd(M_dec);

J_dcd_s = J_dcd(2);
M_dcd_s = M_dcd(2);