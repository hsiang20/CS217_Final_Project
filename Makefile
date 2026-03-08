DESIGN_NAME ?= Top
AWS_DESIGN_NAME ?= design_top
SUBMISSION_NAME ?= lab4-submission

REPO_TOP := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
SRC_HOME := $(REPO_TOP)/src
HLS_HOME := $(REPO_TOP)/hls
REPORT_HOME := $(REPO_TOP)/reports
AWS_HOME := $(REPO_TOP)/$(AWS_DESIGN_NAME)
AWS_LOGS := $(AWS_HOME)/logs

.PHONY: systemc_sim hls_sim hls_sim_debug clean submission copy_rtl hls

# Copy the generated RTL to AWS design folder
copy_rtl:
	mkdir -p $(AWS_HOME)/design
	mkdir -p $(REPORT_HOME)
	mkdir -p $(REPORT_HOME)/hls
	cp $(HLS_HOME)/$(DESIGN_NAME)/Catapult/$(DESIGN_NAME).v1/concat_$(DESIGN_NAME).v $(AWS_HOME)/design
	cp $(HLS_HOME)/$(DESIGN_NAME)/Catapult/$(DESIGN_NAME).v1/$(DESIGN_NAME).rpt $(REPORT_HOME)/hls/
	cp $(HLS_HOME)/$(DESIGN_NAME)/Catapult/$(DESIGN_NAME).v1/scverify/concat_sim_$(DESIGN_NAME)_v_vcs/sim.log $(REPORT_HOME)/hls/$(DESIGN_NAME)_hls_sim.log

# Run SystemC simulation
systemc_sim:
	cd $(SRC_HOME)/$(DESIGN_NAME) && make

# Run HLS and copy RTL
hls: hls_sim copy_rtl

# Run HLS simulation
hls_sim:
	cd $(HLS_HOME)/$(DESIGN_NAME) && make hls

# Run HLS simulation with debug
hls_sim_debug:
	cd $(HLS_HOME)/$(DESIGN_NAME) && make vcs_debug
	verdi -ssf $(HLS_HOME)/$(DESIGN_NAME)/default.fsdb \
		-dbdir $(HLS_HOME)/$(DESIGN_NAME)/Catapult/$(DESIGN_NAME).v1/scverify/concat_sim_$(DESIGN_NAME)_v_vcs/sc_main.daidir/

# Clean all generated files
clean:
	cd $(SRC_HOME)/Top/PEPartition && make clean
	cd $(SRC_HOME)/Top/PEPartition/PEModule/ActUnit && make clean
	cd $(SRC_HOME)/Top/PEPartition/PEModule/PECore && make clean
	cd $(SRC_HOME)/Top/PEPartition/PEModule/PECore/Datapath && make clean
	cd $(SRC_HOME)/Top/PEPartition/PEModule && make clean
	cd $(SRC_HOME)/Top/GBPartition && make clean
	cd $(SRC_HOME)/Top/GBPartition/GBModule/GBCore && make clean
	cd $(SRC_HOME)/Top/GBPartition/GBModule/NMP && make clean
	cd $(SRC_HOME)/Top/GBPartition/GBModule && make clean
	cd $(SRC_HOME)/Top/GBPartition/GBModule/GBControl && make clean
	cd $(SRC_HOME)/Top && make clean
	cd $(HLS_HOME)/Top/PEPartition && make clean
	cd $(HLS_HOME)/Top/PEPartition/PEModule/ActUnit && make clean
	cd $(HLS_HOME)/Top/PEPartition/PEModule/PECore && make clean
	cd $(HLS_HOME)/Top/PEPartition/PEModule && make clean
	cd $(HLS_HOME)/Top/GBPartition && make clean
	cd $(HLS_HOME)/Top/GBPartition/GBModule/GBCore && make clean
	cd $(HLS_HOME)/Top/GBPartition/GBModule/NMP && make clean
	cd $(HLS_HOME)/Top/GBPartition/GBModule && make clean
	cd $(HLS_HOME)/Top/GBPartition/GBModule/GBControl && make clean
	cd $(HLS_HOME)/Top && make clean
	rm -rf design_top/build/checkpoints/
	rm -rf design_top/build/constraints/generated_cl_clocks_aws.xdc
	rm -rf design_top/build/reports/
	rm -rf design_top/build/src_post_encryption
	rm -rf design_top/build/scripts/hd_visual/
	rm -rf design_top/build/scripts/.Xil/
	rm -rf design_top/build/scripts/*.jou
	rm -rf design_top/build/scripts/*.vivado.log
	rm -rf design_top/build/scripts/*.txt
	rm -rf design_top/verif/sim/
	rm -rf design_top/software/runtime/design_top
	rm -rf ./*~
	rm -rf ./*.key
	rm -rf ./core*
	rm -rf ./Catapult*
	rm -rf ./catapult*
	rm -rf ./*.log
	rm -rf ./design_checker_*.tcl
	rm -rf ./DVE*
	rm -rf ./verdi*
	rm -rf ./slec*
	rm -rf ./novas*
	rm -rf ./*.fsdb
	rm -rf ./*.saif*
	rm -rf ./*.vpd

# Create submission archive
submission:
	zip -r $(SUBMISSION_NAME).zip \
		src/$(DESIGN_NAME)/PEPartition/PEModule/PEModule.h \
		src/$(DESIGN_NAME)/PEPartition/PEPartition.h \
		src/$(DESIGN_NAME)/GBPartition/GBModule/GBModule.h \
		src/$(DESIGN_NAME)/GBPartition/GBPartition.h \
		src/$(DESIGN_NAME)/Top.h \
		design_top/design/concat_$(DESIGN_NAME).v \
		design_top/design/design_top.sv \
		$(REPORT_HOME) \
