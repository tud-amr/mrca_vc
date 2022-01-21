function h = plot_mean_std(ax, mean, std, lineColor, lineWidth, varargin)
    
    n_data = length(mean);
    xN = 1 : n_data;
    h_mean = plot(ax, xN, mean, 'LineWidth', lineWidth, ...
        'Color', lineColor);
%     h_mean.Color(4) = 0.6;
    
    for i = 1 : n_data-1
        xP = [i, i+1, i+1, i];
        yP = [mean(i)-std(i), mean(i+1)-std(i+1), ...
            mean(i+1)+std(i+1), mean(i)+std(i)];
        fill(ax, xP, yP, lineColor, varargin{:});
    end
    
    h = h_mean;

end