function output = mixedNorm(X, pq)

[~,columns] =size(X);
n=zeros(columns,1);

if pq==[2 'inf']
    for i=1:columns
        n(i) = norm(X(:,i));
    end
    output = norm(n,'inf');
elseif pq==['inf' 2]
    for i=1:columns
        n(i) = norm(X(:,i),"inf");
    end
    output = norm(n);
else
    output = NaN;
    msgbox('wtf did you asked');
end

end