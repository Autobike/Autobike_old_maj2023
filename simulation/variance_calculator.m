function [variance] = variance_calculator(real,estimated,flag)
% flag=1: sample the data with 1Hz frequency when GPS update and start with
% the same time instant.
% Cut of the vector data every 30 second where data are significant.
if flag == 1
        variable_estimated_cut=estimated(8:10:3033);
        variable_real_cut=real(1:10:3025);
        
else
        variable_estimated_cut=estimated(8:1:3033);
        variable_real_cut=real(1:1:3025);
end
% variance between estimation and measurement
variable_estimated_cut=estimated(8:10:3033);
variable_real_cut=real(1:10:end);
N=size(variable_estimated_cut);
diff=variable_real_cut-variable_estimated_cut;
Mean=mean(diff);
variance=(1/N(1))*sum((Mean-diff).^2);

end