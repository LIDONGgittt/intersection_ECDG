#!/bin/bash
tar_ball_name=sumosim_backup-$(date +"%Y%m%d%H%M%S").tar.gz
target_dir=sumosim

echo "back up in file: backup/$tar_ball_name"
tar -czf $tar_ball_name $target_dir

mkdir backup
mv $tar_ball_name backup/
