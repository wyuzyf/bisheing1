function pd = posdef(M)

[T,p]=chol(M);
pd = (p==0);
