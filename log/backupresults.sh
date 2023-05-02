tar_ball_name=Arc_sumosim-$(date +"%Y%m%d%H%M%S")
target_dir=sumosim

tar -czf $tar_ball_name $target_dir
mv $tar_ball_name backup/
