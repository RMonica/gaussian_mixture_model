/*
 * Copyright (c) 2014, Riccardo Monica
 *   RIMLab, Department of Information Engineering, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GMM_RVIZ_CONVERTER_H
#define GMM_RVIZ_CONVERTER_H

#define PARAM_NAME_INPUT_TOPIC     "input_topic"
#define PARAM_DEFAULT_INPUT_TOPIC  "/gmm_rviz_converter_input"

#define PARAM_NAME_OUTPUT_TOPIC    "output_topic"
#define PARAM_DEFAULT_OUTPUT_TOPIC "/gmm_rviz_converter_output"

#define PARAM_NAME_FRAME_ID        "frame_id"
#define PARAM_DEFAULT_FRAME_ID     "/root"

#define PARAM_NAME_SCALE           "scale"
#define PARAM_DEFAULT_SCALE        (double(10.0))

// if true, relative size of gaussians based on eigenvalues will be disregarded
#define PARAM_NAME_NORMALIZE       "normalize"
#define PARAM_DEFAULT_NORMALIZE    (bool(false))

// this is used to clear rviz before sending more markers
#define PARAM_NAME_MAX_MARKERS     "max_markers"
#define PARAM_DEFAULT_MAX_MARKERS  (int(1000))

#define PARAM_NAME_RVIZ_NAMESPACE  "rviz_namespace"
#define PARAM_DEFAULT_RVIZ_NAMESPACE "gmm_rviz_converter"

// this parameter must contain 3 space separated values, for each dimension x, y, z
#define PARAM_NAME_COORD_MASK     "coordinate_mask"
#define PARAM_DEFAULT_COORD_MASK  "index 1 index 2 index 3"
#define PARAM_CMD_CONST            "const"
// if the value is in the format "const r", where r is real
//   the coordinate is constant at value r
#define PARAM_CMD_INDEX            "index"
// if the value is in the format "index n", where n is a non-negative integer,
//   the coordinate will be read from the gaussian mixture at position n

// This is the number of coordinates in 3D space. Hope this won't change anytime soon.
#define NUM_OUTPUT_COORDINATES 3

#endif // GMM_RVIZ_CONVERTER_H
