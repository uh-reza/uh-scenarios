FILE(REMOVE_RECURSE
  "../src/face_detector/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/face_detector/msg/__init__.py"
  "../src/face_detector/msg/_FaceDetectorAction.py"
  "../src/face_detector/msg/_FaceDetectorGoal.py"
  "../src/face_detector/msg/_FaceDetectorActionGoal.py"
  "../src/face_detector/msg/_FaceDetectorResult.py"
  "../src/face_detector/msg/_FaceDetectorActionResult.py"
  "../src/face_detector/msg/_FaceDetectorFeedback.py"
  "../src/face_detector/msg/_FaceDetectorActionFeedback.py"
  "../msg/FaceDetectorAction.msg"
  "../msg/FaceDetectorGoal.msg"
  "../msg/FaceDetectorActionGoal.msg"
  "../msg/FaceDetectorResult.msg"
  "../msg/FaceDetectorActionResult.msg"
  "../msg/FaceDetectorFeedback.msg"
  "../msg/FaceDetectorActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
