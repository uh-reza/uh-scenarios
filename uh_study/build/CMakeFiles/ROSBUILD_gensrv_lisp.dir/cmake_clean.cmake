FILE(REMOVE_RECURSE
  "../src/uh_study/msg"
  "../src/uh_study/srv"
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/AddTwoInts.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_AddTwoInts.lisp"
  "../srv_gen/lisp/FacePoseSrv.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_FacePoseSrv.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
