% generate a random covariance matrix: symmetric PD matrix
% this snippet is from http://math.stackexchange.com/questions/357980/matlab-code-for-generating-random-symmetric-positive-definite-matrix
% the idea is that any symmetric PD matrix A can be written as A=Q'DQ,
% where Q is a random matrix and D is a diagonal matrix with positive
% diagonal elements

function A = randCov(n,x,y)

Q = randn(n,n);

% can be made anything, even zero 
% used to shift the mode of the distribution
eigen_mean = (x/10)^2+(y/10)^2; 

A = Q' * diag(abs(eigen_mean+randn(n,1))) * Q;

return 

%%% another choice from the same webpage is
% the issue is that the generated A may not be a covariance matrix
% sometimes.
% function A = generateSPDmatrix(n)
% % Generate a dense n x n symmetric, positive definite matrix
% 
% A = rand(n,n); % generate a random n x n matrix
% 
% % construct a symmetric matrix using either
% A = A+A'; OR
% A = A*A';
% % The first is significantly faster: O(n^2) compared to O(n^3)
% 
% % since A(i,j) < 1 by construction and a symmetric diagonally dominant matrix
% %   is symmetric positive definite, which can be ensured by adding nI
% A = A + n*eye(n);
% 
% end