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

#ifndef GMM_NODE_H
#define GMM_NODE_H

// minimum number of gaussians
#define PARAM_NAME_GAUSSIAN_COUNT_MIN    "gaussian_count_min"
#define PARAM_DEFAULT_GAUSSIAN_COUNT_MIN 1

// search will terminate when the gaussian count reaches this OR...
#define PARAM_NAME_GAUSSIAN_COUNT_MAX    "gaussian_count_max"
#define PARAM_DEFAULT_GAUSSIAN_COUNT_MAX 10
// ...when the current BIC - lower BIC is higher than this
#define PARAM_NAME_BIC_TERM_THRESHOLD    "bic_termination_threshold"
#define PARAM_DEFAULT_BIC_TERM_THRESHOLD (double(1000000.0))

// the algorithm will initialize the EM model by splitting this column
#define PARAM_NAME_TIME_COLUMN_ID        "time_column_id"
#define PARAM_DEFAULT_TIME_COLUMN_ID     0

// decrease this to get more gaussians, increase to get less
#define PARAM_NAME_BIC_PARAMS_WEIGHT     "bic_params_weight"
#define PARAM_DEFAULT_BIC_PARAMS_WEIGHT  1.0

// input topics
// this is a Float32MultiArray
// with two dimensions, rows and columns
// each row contains a sample, each column a dimension
#define PARAM_NAME_DATA_INPUT_TOPIC      "data_input_topic"
#define PARAM_DEFAULT_DATA_INPUT_TOPIC   "/gmm/data"

// output topics
// outputs a gaussian_mixture_model/GaussianMixture.msg
#define PARAM_NAME_MIX_OUTPUT_TOPIC      "mix_output_topic"
#define PARAM_DEFAULT_MIX_OUTPUT_TOPIC   "/gmm/mix"

#endif // GMM_NODE_H
