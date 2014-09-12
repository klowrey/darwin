#!/bin/bash

FILES="$(ls *.csv | grep -v '\Log')"

for f in $FILES 
do
	scp $f klowrey@superman.cs.washington.edu:~/Dropbox/robotics/darwin/Playback/trajectory/../logs
	ssh klowrey@superman.cs.washington.edu "python ~/Dropbox/robotics/darwin/Playback/darwin_log2sabot.py ~/Dropbox/robotics/darwin/Playback/trajectory/../logs/$f"
done
