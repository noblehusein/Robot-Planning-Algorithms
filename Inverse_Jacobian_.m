m1 = 1;
m2 = 1;
t1 = pi/4;
t2 = pi/8;
s1 = sin(t1);
c1 = cos(t1);
s12 = sin(t1+t2);
c12 = cos(t1+t2);
m11 = -m1*s1-m2*s12;
m12 = -m2*s12;
m21 = m1*c1 + m2*c12;
m22 = m2*c12;

jacobian  = [m11 m12 0 0; m21 m22 0 0 ;0 0 -1 0;0 0 0 0;0 0 0 0;1 1 0 -1];

 M  = svd(jacobian);
 [U T P]= svd(jacobian);
 n = nnz(M);
 
 pn = zeros(n,n); 
 for i=1:n
     for j=1:n
         if i ==j
             pn(i,j) = M(i);
         else
             pn(i,j) = 0;
         end
     end
 end
 
 M_inv = inv(pn);
 epsilon = 0.1;
 for i=1:n
     for j =1:n
         if (M(n)/M(i)) <epsilon
             M_inv(i,j) = 0;
         end
     end
 end
 
 mfinal = zeros(6,n);
 
 for i=1:n
     for j =1:n
         mfinal(i,j)= M_inv(i,j);
     end
 end
 
 
 psudeo_J = U*mfinal*P
 
      
         