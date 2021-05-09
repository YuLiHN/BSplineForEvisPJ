function result = RCoxdeBoor(i,k,knots,t)
if(k==0)
    if(knots(i)<=t && t<knots(i+1))
        result=1;
        return;
    else
        result=0;
        return;
    end
else
    if(knots(i+k)-knots(i)==0)
        w1=0;
    else
        w1=(t-knots(i))/(knots(i+k)-knots(i));
    end
    if(knots(i+k+1)-knots(i+1)==0)
        w2=0;
    else
        w2=(knots(i+k+1)-t)/(knots(i+k+1)-knots(i+1));
    end
end
result=w1*RCoxdeBoor(i,k-1,knots,t)+w2*RCoxdeBoor(i+1,k-1,knots,t);