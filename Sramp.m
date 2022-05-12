function[r,n]=Sramp(amax,vmax,xf,T)
%---Creates a uniform sampled profile between 
%   0 and a final position, xf, with specified maximum
%   acceleration, amax, and velocity, vmax.

t1=vmax/amax;
x1=1/2*amax*t1*t1;
if (x1<xf/2)
    xr=xf-2*x1;
    tr=xr/vmax;
    t2=t1+tr;
    tf=t2+t1;
else
    x1=xf/2;
    t1=sqrt(2*x1/amax);
    t2=t1;
    tf=2*t1;
end
n=fix(tf/T);
for i=0:n+1
    t=i*T;
    if(t<t1)
        r(i+1)=1/2*amax*t*t;
    elseif (t>=t1 && t<t2)
        r(i+1)=x1+vmax*(t-t1);
    elseif (t>=t2 && t<tf)
        r(i+1)=xf-1/2*amax*(tf-t)*(tf-t);
    else
        r(i+1)=xf;
    end
end
