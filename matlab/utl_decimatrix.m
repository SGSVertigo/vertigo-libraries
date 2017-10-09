function [ output_data ] = utl_decimatrix( input_data, deciratio )
%UTL_DECIMATRIX Decimates a 2D matrix.
%   Ben Kaye
    output_data = zeros(ceil(size(input_data, 1) / deciratio), size(input_data, 2)); 
    for col = 1:size(input_data, 2)
      output_data(:, col) = decimate(input_data(:, col), deciratio);
    end
end

