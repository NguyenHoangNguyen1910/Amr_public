# Saved maps

`./save_map.sh <name>` writes the real robot's occupancy map and serialized
SLAM pose graph here. The `.yaml` plus `.pgm` pair stores the occupancy map;
the `.posegraph` plus `.data` pair is used by `slam_toolbox` to localize the
real robot in the same environment.

Do not rename only one file in a `.yaml`/`.pgm` pair: the YAML contains the
relative image filename.
