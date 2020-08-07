function [ MM ] = krone( A,B )
%KRON Summary of this function goes here
%   Detailed explanation goes here

[a1,a2]=size(A);
[b1,b2]=size(B);

MM=zeros(a1*b1,a2*b2);

    for i=1:a1
        for j=1:a2
            MM((b1*(i-1)+1):(b1*i),(b2*(j-1)+1):(b2*j))=A(i,j)*B;
        end
    end

end

