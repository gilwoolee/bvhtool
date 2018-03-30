This is a toolkit for visualizing our data in linux for testing. This uses [bvhplay](https://sites.google.com/a/cgspeed.com/cgspeed/bvhplay), which visualizes bvh files. Since our data is generated in csv files, we provide additional conversion funtions to generate bvh files.

To utilize this tool, install the following:

# Install cgkit
 - go to cgkit
 - follow README to install cgkit. You'll first need to install scons by `sudo apt-get install scons`.

# Install BVHToolkit
 - go to BTHToolkit
 - ```python setup.py install```

# Test if bvhplay works
 - ```python bvhplay/bvhplay.py```
 - Open skeleton.bvh. This should show a static skeleton. 

# Convert csv files to bvhfiles
 - ```python csv2bvh/csv2bvh.py -f test_taylor_5s.csv```
This produces `test_taylor_5s.bvh`, which can be played by the bvhplayer.

Use `s` to zoom out, `w` to zoom in, `a` and `d` to rotate.

