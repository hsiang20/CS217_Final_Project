# Copyright (c) 2016-2019, NVIDIA CORPORATION.  All rights reserved.
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

LOGFILE = build.log
CFLAGS = -DHLS_ALGORITHMICC
DEBUG_FLAG = -DDEBUG_LEVEL=5
HLS_SCRIPTS ?= $(REPO_TOP)/scripts/hls/

include $(HLS_SCRIPTS)/Makefile_src

.PHONY: all run matmul matmul_run

all: clean sim_test run

run:
	./sim_test

sim_test: $(wildcard *.h) testbench.cpp
	$(CC) -o sim_test $(CFLAGS) $(USER_FLAGS) testbench.cpp $(BOOSTLIBS) $(LIBS) > $(LOGFILE) 2>&1

sim_test_debug: $(wildcard *.h) testbench.cpp
	$(CC) -o sim_test $(CFLAGS) $(DEBUG_FLAG) $(USER_FLAGS) testbench.cpp $(BOOSTLIBS) $(LIBS) > $(LOGFILE) 2>&1

matmul: clean sim_test_matmul matmul_run

matmul_run:
	./sim_test_matmul

sim_test_matmul: $(wildcard *.h) testbench_matmul.cpp
	$(CC) -o sim_test_matmul $(CFLAGS) $(USER_FLAGS) testbench_matmul.cpp $(BOOSTLIBS) $(LIBS) > $(LOGFILE) 2>&1