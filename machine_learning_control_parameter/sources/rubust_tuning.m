 
 sys=tf(145.28,[7.9423e-04,2*0.059068*0.028182,1,0]);
dsys=c2d(sys,ts,'z');

k_un = ureal('k_un',145,'Percentage',60);
order_1 = ureal('order_1',1,'Percentage',40);
order_2 = ureal('order_2',2*0.059068*0.028182,'Percentage',40);
order_3 = ureal('order_3',7.9423e-04,'Percentage',40);

  sys=tf(k_un,[order_3,order_2,order_1,0])
  dsys=c2d(sys,ts,'zoh')