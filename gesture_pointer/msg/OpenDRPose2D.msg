# Copyright 2020-2023 OpenDR European Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# This message represents a full OpenDR human pose 2D as a list of keypoints

std_msgs/Header header

# The id of the pose
int32 pose_id

# The pose detection confidence of the model
float32 conf

# A list of a human 2D pose keypoints
OpenDRPose2DKeypoint[] keypoint_list