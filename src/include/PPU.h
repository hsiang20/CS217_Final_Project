/*
 * All rights reserved - Stanford University. 
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License.  
 * You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#ifndef PPU_H
#define PPU_H

#include <systemc.h>
#include <nvhls_int.h>
#include <nvhls_types.h>
#include <nvhls_vector.h>
#include <nvhls_module.h>
#include <ac_math/ac_sigmoid_pwl.h>
#include <ac_math/ac_tanh_pwl.h>
  

// Important Update: change everything to vector type
// spec::kNumVectorLanes
// spec::ActVectorType
// spec::VectorType
//       #pragma hls_unroll yes



#pragma hls_design ccore
#pragma hls_ccore_type combinational
inline void Tanh (const spec::ActVectorType in, spec::ActVectorType& out)
{
  spec::ActVectorType out_tmp;
  #pragma hls_unroll yes
  for (int i = 0; i < spec::kNumVectorLanes; i++) {
    ac_fixed<spec::kActWordWidth, spec::kActWordWidth - spec::kActNumFrac, true> in_ac;
    in_ac.set_slc(0, in[i]);
    ac_fixed<spec::kActWordWidth, 2, true> out_ac;
    ac_math::ac_tanh_pwl(in_ac, out_ac);
    ac_fixed<spec::kActWordWidth, spec::kActWordWidth - spec::kActNumFrac, true> result;
    result = out_ac;
    out_tmp[i] = result.template slc<spec::kActWordWidth>(0);
  }
  out = out_tmp;
}

#pragma hls_design ccore
#pragma hls_ccore_type combinational
inline void Relu (const spec::ActVectorType in, spec::ActVectorType& out) 
{
  spec::ActVectorType out_tmp;   
  #pragma hls_unroll yes
  for (int i = 0; i < spec::kNumVectorLanes; i++) {  
    if (in[i] < 0) out_tmp[i] = 0;
    else out_tmp[i] = in[i];
  }
  out = out_tmp;
}  

#pragma hls_design ccore
#pragma hls_ccore_type combinational
inline void Silu (const spec::ActVectorType in, spec::ActVectorType& out) 
{
  spec::ActVectorType out_tmp;
  #pragma hls_unroll yes
  for (int i = 0; i < spec::kNumVectorLanes; i++) {
    ac_fixed<spec::kActWordWidth, spec::kActWordWidth - spec::kActNumFrac, true> in_ac;
    in_ac.set_slc(0, in[i]);
    ac_fixed<spec::kActWordWidth, 1, false> sigmoid_out_ac;
    ac_math::ac_sigmoid_pwl(in_ac, sigmoid_out_ac);
    ac_fixed<spec::kActWordWidth * 2, spec::kActWordWidth, true> mul_out_ac = in_ac * sigmoid_out_ac;
    ac_fixed<spec::kActWordWidth, spec::kActWordWidth - spec::kActNumFrac, true> result;
    result = mul_out_ac;
    out_tmp[i] = result.template slc<spec::kActWordWidth>(0);
  }
  out = out_tmp;
}

#pragma hls_design ccore
#pragma hls_ccore_type combinational
inline void Gelu (const spec::ActVectorType in, spec::ActVectorType& out) 
{
  spec::ActVectorType out_tmp;
  #pragma hls_unroll yes
  for (int i = 0; i < spec::kNumVectorLanes; i++) {
    ac_fixed<spec::kActWordWidth, spec::kActWordWidth - spec::kActNumFrac, true> in_ac;
    in_ac.set_slc(0, in[i]);
    // Using a simple approximation for GELU for HLS, as ac_math does not provide it.
    // GELU(x) approx x * sigmoid(1.702 * x)
    ac_fixed<4, 2, false> scale_factor = 1.702;
    ac_fixed<spec::kActWordWidth + 4, spec::kActWordWidth - spec::kActNumFrac + 2, true> scaled_in = in_ac * scale_factor;
    ac_fixed<spec::kActWordWidth, 1, false> sigmoid_out_ac;
    ac_math::ac_sigmoid_pwl(scaled_in, sigmoid_out_ac);
    ac_fixed<spec::kActWordWidth * 2, spec::kActWordWidth, true> mul_out_ac = in_ac * sigmoid_out_ac;
    ac_fixed<spec::kActWordWidth, spec::kActWordWidth - spec::kActNumFrac, true> result;
    result = mul_out_ac;
    out_tmp[i] = result.template slc<spec::kActWordWidth>(0);
  }
  out = out_tmp;
}  

#endif
