
function [out, in_old, out_old ] = discreate_function(in,num ,den, in_old,out_old)
%n½×ÀëÉ¢±í´ï
 this_len  = length(den);
    for ii=2:this_len
        in_old(this_len-ii+2) =  in_old(this_len-ii+1);
    end
    in_old(1) =in;
     for ii=2:this_len
        out_old(this_len-ii+2) =  out_old(this_len-ii+1);
     end
 
    out =  0;
    for ii=2:this_len
        out =  out - den(ii)*out_old(ii);
    end
    for ii=1:this_len
        out =  out + num(ii)*in_old(ii);
    end
%       out = ABS_LIMIT(out,7);
    out_old(1) = out ;
end



function out=ABS_LIMIT(x,limit)
 if(x<(-limit))
     out = -limit;
 elseif(x>limit)
     out = limit;
 else
     out = x;
 end
return;
end
