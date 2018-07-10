function out=LIMIT(x,left,right)
 if(x<left)
     out = left;
 elseif(x>right)
     out = right;
 else
     out = x;
 end
return;
end