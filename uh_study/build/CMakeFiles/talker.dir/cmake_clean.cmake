FILE(REMOVE_RECURSE
  "../src/uh_study/msg"
  "../src/uh_study/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/talker.dir/src/talker.o"
  "../bin/talker.pdb"
  "../bin/talker"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/talker.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
