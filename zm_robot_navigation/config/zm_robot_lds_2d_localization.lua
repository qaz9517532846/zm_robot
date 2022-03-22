include "zm_robot_lds_2d.lua"

TRAJECTORY_BUILDER.pure_localization = true
POSE_GRAPH.optimize_every_n_nodes = 20

return options