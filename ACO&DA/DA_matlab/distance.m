%___________________________________________________________________%
%  Dragonfly Algorithm (DA) source codes demo version 1.0           %
%                                                                   %
%  Developed in MATLAB R2011b(7.13)                                 %
%                                                                   %
%  Author and programmer: Seyedali Mirjalili                        %
%                                                                   %
%         e-Mail: ali.mirjalili@gmail.com                           %
%                 seyedali.mirjalili@griffithuni.edu.au             %
%                                                                   %
%       Homepage: http://www.alimirjalili.com                       %
%                                                                   %
%   Main paper:                                                     %
%                                                                   %
%   S. Mirjalili, Dragonfly algorithm: a new meta-heuristic         %
%   optimization technique for solving single-objective, discrete,  %
%    and multi-objective problems, Neural Computing and Applications% 
%   DOI: http://dx.doi.org/10.1007/s00521-015-1920-1                %
%                                                                   %
%___________________________________________________________________%

% This function calculate the Euclidean distance 

function o = distance(a,b)

for i=1:size(a,1)
    o(1,i)=sqrt((a(i)-b(i))^2);
end
