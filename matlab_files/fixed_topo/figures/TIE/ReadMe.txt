old_file contain files that use 0.5 for bearing sensor covariance and 1 for range sensor covariance.

files in the current folder use 0.5 for bearing sensor covariance and 5 for range sensor covariance.

file name with suffix v001 is the metrics data when target motion model noise covariance is 0.01.
otherwise, no motion noise is added to the target.

the pdf files in this folder is to be used in the paper. the ones with _noise is the results when target motion is stochastic. otherwise, the target motion is deterministic.

process_plot contains data that are used to draw the localization process plot. draw_progress_plots.ipynb is the notebook for drawing.

metrics_plot contains data for drawing the comparison plot of DBF, CbDF, CF. draw_metric_plots.ipynb is the notebook for drawing.

colomaputil.py is used to call my colormap.