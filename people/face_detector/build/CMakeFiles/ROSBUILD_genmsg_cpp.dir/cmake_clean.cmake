FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/face_detector/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/face_detector/FaceDetectorAction.h"
  "../msg_gen/cpp/include/face_detector/FaceDetectorGoal.h"
  "../msg_gen/cpp/include/face_detector/FaceDetectorActionGoal.h"
  "../msg_gen/cpp/include/face_detector/FaceDetectorResult.h"
  "../msg_gen/cpp/include/face_detector/FaceDetectorActionResult.h"
  "../msg_gen/cpp/include/face_detector/FaceDetectorFeedback.h"
  "../msg_gen/cpp/include/face_detector/FaceDetectorActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
