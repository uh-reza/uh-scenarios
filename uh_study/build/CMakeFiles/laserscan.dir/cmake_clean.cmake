FILE(REMOVE_RECURSE
  "../src/uh_study/msg"
  "../src/uh_study/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/laserscan.dir/src/laserscan.o"
  "CMakeFiles/laserscan.dir/src/LegsDetector.o"
  "../bin/laserscan.pdb"
  "../bin/laserscan"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/laserscan.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
