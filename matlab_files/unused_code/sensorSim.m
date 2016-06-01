function z = sensorSim(rbt,fld)
x_r = [fld.x;fld/y];
t = [fld.tx;fld.ty];
inv_cov = rbt.inv_sen_cov;
offset = rbt.offset;

prob = exp(-1/2*(t+offset-x_r)'*inv_cov*(t+offset-x_r));
z = (rand(1,1) < prob);
end