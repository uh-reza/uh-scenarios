FILE(REMOVE_RECURSE
  "../src/uh_study/msg"
  "../src/uh_study/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/LaserScanToPointCloud.dir/src/LaserScanToPointCloud.o"
  "CMakeFiles/LaserScanToPointCloud.dir/src/LegsDetector.o"
  "../bin/LaserScanToPointCloud.pdb"
  "../bin/LaserScanToPointCloud"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/LaserScanToPointCloud.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
